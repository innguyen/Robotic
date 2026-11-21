"""Desktop simulation of the production navigation core; no ROS connection."""
import math
import argparse
from concurrent.futures import wait
from pathlib import Path
import numpy as np

from collision_monitor_ros1 import Config, Monitor, scan_points
from navigation_core_ros1 import Navigator, NavConfig, to_world
from map_core_ros1 import MappedNavigator, OccupancyMap
from navigation_ros1 import MONITOR_OK, STANDOFF
from search_core_ros1 import SearchMission


HERE = Path(__file__).resolve().parent
# Cluttered room, similar to the robot screenshots: walls plus furniture (metres).
ROOM = [[-1.0, 1.4, 3.0, 1.5], [-1.0, -1.5, 3.0, -1.4], [2.9, -1.5, 3.0, 1.5],
        [-1.1, -1.5, -1.0, 1.5], [.5, .3, .8, .7], [.6, -.6, .9, -.2],
        [1.4, -.1, 1.7, .25], [2.0, .6, 2.4, 1.0]]


def robot_config():
    """Same values the robot loads from the YAML files (scan_frame etc. ignored)."""
    import yaml
    def load(name, keys):
        data = yaml.safe_load((HERE / name).read_text(encoding='utf-8')) or {}
        return {key: value for key, value in data.items() if key in keys}
    return (NavConfig(**load('navigation_ros1.yaml', NavConfig.DEFAULTS)),
            Config(**load('collision_monitor_ros1.yaml', Config.DEFAULTS)))


class PersonBox(list):
    """Obstacle box that marks the simulated person (LiDAR sees it too)."""
    person = True


class Simulation:
    def __init__(self, gap=.60, scenario='gap', config=None, persistent_map=False):
        nav_cfg, cc = config or (NavConfig(), Config())
        # Same as navigation_ros1.py: planning runs in a worker thread (PLANNING states).
        self.nav = MappedNavigator(nav_cfg, cc, async_planning=True)
        self.exact_goal, self.goal_error = False, ''
        self.monitor = Monitor(self.nav.cc)
        self.pose = np.array([0., 0., 0.])
        self.target = np.array([2.1, 0.])
        self.time = 10.
        self.trail = [self.pose[:2].copy()]
        self.state, self.monitor_state = 'READY', 'IDLE'
        self.command = self.checked = (0., 0., 0.)
        # Rectangles: xmin, ymin, xmax, ymax (metres).
        self.boxes = ([[.8, gap/2, 1.2, gap/2+.45],
                       [.85, -gap/2-.35, 1.1, -gap/2]] if scenario == 'gap'
                      else [[.8, -.16, 1.15, .16]] if scenario == 'detour'
                      else [list(box) for box in ROOM] if scenario == 'room' else [])
        if scenario in ('room', 'search'):
            self.target = np.array([2.5, 0.])
        self.angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        self.person, self.search, self.seen_steps, self.seen_at = None, None, 0, None
        self.ranges = None
        if scenario == 'search':
            self.boxes = [list(box) for box in ROOM]
            self.start_search((2.4, -.95))
        self.observe()
        if persistent_map:
            self.install_persistent_map()

    def install_persistent_map(self, bounds=(-1.2, -1.6, 3.2, 1.6)):
        """Known simulated floor plan. The person is a dynamic obstacle, never baked in."""
        res = self.nav.cfg.resolution
        origin = np.floor(np.asarray(bounds[:2])/res)*res
        size = np.ceil((np.asarray(bounds[2:])-origin)/res).astype(int)
        rows, cols = np.indices((size[1], size[0]))
        xs, ys = origin[0]+(cols+.5)*res, origin[1]+(rows+.5)*res
        raw = np.zeros(rows.shape, dtype=np.int8)
        for box in self.boxes:
            if not getattr(box, 'person', False):
                # Intersect whole pixels with furniture, including thin walls.
                occupied = ((xs+res/2 >= box[0]) & (xs-res/2 <= box[2]) &
                            (ys+res/2 >= box[1]) & (ys-res/2 <= box[3]))
                raw[occupied] = 100
        self.nav.set_global_map(OccupancyMap(raw, res, (*origin, 0.)))
        self.nav.grid = self.nav.costmap.snapshot(self.pose, self.time)

    def start_search(self, person):
        """Search mission (search_core_ros1) for a person standing at `person`."""
        self.boxes = [box for box in self.boxes if not getattr(box, 'person', False)]
        self.person = None if person is None else np.asarray(person, dtype=float)
        if self.person is not None:
            self.boxes.append(PersonBox([self.person[0]-.15, self.person[1]-.15,
                                         self.person[0]+.15, self.person[1]+.15]))
        self.nav.reset()
        self.search = SearchMission(self.nav)
        self.seen_steps, self.seen_at = 0, None

    def camera_target(self):
        """Idealised YOLO + tracker: person inside the camera view and not hidden behind a
        box for 2 frames confirms it; the position is then remembered for 2.5 s."""
        if self.person is None:
            return None
        offset = self.person-self.pose[:2]
        distance = float(np.linalg.norm(offset))
        bearing = (math.atan2(offset[1], offset[0])-self.pose[2]+math.pi) % (2*math.pi)-math.pi
        visible = abs(bearing) <= math.radians(self.search.cfg.camera_fov_deg)/2 and distance <= self.search.cfg.camera_range
        if visible and self.ranges is not None:
            index = int(np.argmin(np.abs((self.angles-bearing+math.pi) % (2*math.pi)-math.pi)))
            visible = self.ranges[index] >= distance-.3   # first LiDAR hit is the person itself
        self.seen_steps = self.seen_steps+1 if visible else 0
        if self.seen_steps >= 2:
            self.seen_at = self.time
        if self.seen_at is not None and self.time-self.seen_at <= 2.5:
            return self.person.copy()
        return None

    def scan(self):
        cc = self.nav.cc
        origin = to_world([[cc.scan_x, cc.scan_y]], self.pose)[0]
        angles = self.angles+self.pose[2]+cc.scan_yaw
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        distances = np.full(len(angles), np.inf)
        for box in self.boxes:
            lo, hi = np.array(box[:2]), np.array(box[2:])
            near, far = np.full(len(angles), -np.inf), np.full(len(angles), np.inf)
            valid = np.ones(len(angles), dtype=bool)
            for axis in range(2):
                parallel = np.abs(directions[:, axis]) < 1e-12
                valid &= ~parallel | ((origin[axis] >= lo[axis]) & (origin[axis] <= hi[axis]))
                divisor = np.where(parallel, 1., directions[:, axis])
                a, b = (lo[axis]-origin[axis])/divisor, (hi[axis]-origin[axis])/divisor
                near = np.maximum(near, np.where(parallel, -np.inf, np.minimum(a, b)))
                far = np.minimum(far, np.where(parallel, np.inf, np.maximum(a, b)))
            hit = valid & (far >= np.maximum(near, 0.))
            distance = np.where(near >= 0., near, far)
            distances = np.minimum(distances, np.where(hit & (distance <= 6.), distance, np.inf))
        return distances

    def observe(self):
        ranges = self.ranges = self.scan()
        self.nav.costmap.update(ranges, -math.pi, 2*math.pi/720, .05, 6., self.pose, self.time)
        if self.search is not None:
            self.search.update_coverage(ranges, -math.pi, 2*math.pi/720, .05, self.pose)
        self.monitor.set_scan(scan_points(ranges, -math.pi, 2*math.pi/720, .05, 6., self.nav.cc), self.time)
        self.nav.grid = self.nav.costmap.snapshot(self.pose, self.time)

    def set_goal(self, point, exact):
        """Vision target (standoff) or map point; map points use the robot's acceptance check."""
        self.nav.reset()
        self.goal_error = ''
        grid = (self.nav.exploration_grid(self.pose, self.time) if self.nav.costmap.global_map is not None
                else self.nav.grid)
        if exact and (grid is None or not grid.inside(point) or grid.known_blocked(point)):
            self.goal_error = 'GOAL_BLOCKED_OR_UNKNOWN'
            return False
        self.target, self.exact_goal = np.asarray(point, dtype=float), exact
        return True

    def step(self, planner_wait=.2):
        self.time += .1
        self.observe()
        if self.monitor_state not in MONITOR_OK:
            # navigation_ros1.gate(): a blocking monitor state stops the command.
            self.command, self.state = (0., 0., 0.), 'MONITOR_'+self.monitor_state
        elif self.search is not None:
            self.command, self.state = self.search.step(self.pose, self.camera_target(),
                STANDOFF['person'], self.time, .1,
                target_visible=self.seen_steps >= 2,
                target_age=self.time-self.seen_at if self.seen_at is not None else float('inf'),
                measured_velocity=self.checked)
        else:
            self.command, self.state = self.nav.step(self.pose, self.target, .35, self.time, .1,
                                                     exact_goal=self.exact_goal)
        if self.nav.plan_future is not None:
            # Simulated time stops while waiting, as if the planner kept up in real time.
            wait([self.nav.plan_future], timeout=planner_wait)
        if self.nav.global_future is not None:
            wait([self.nav.global_future], timeout=planner_wait)
        self.monitor.set_command(*self.command, now=self.time)
        self.checked, self.monitor_state = self.monitor.evaluate(self.time)
        vx, vy, wz = self.checked
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose += np.array([c*vx-s*vy, s*vx+c*vy, wz])*.1
        self.trail.append(self.pose[:2].copy())


def desktop(config, config_name, initial_scenario='gap', persistent_map=False):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title('JetAuto — mô phỏng thuật toán ROS 1 (không kết nối robot) — ' + config_name)
    root.geometry('1140x740')
    controls = ttk.Frame(root, padding=8)
    controls.pack(fill='x')
    scene = tk.StringVar(value=initial_scenario)
    ttk.Label(controls, text='Cảnh:').pack(side='left')
    ttk.Combobox(controls, textvariable=scene, values=['gap', 'detour', 'room', 'search', 'empty'], width=9,
                 state='readonly').pack(side='left')
    gap = tk.DoubleVar(value=60.)
    ttk.Label(controls, text='Khe (cm):').pack(side='left', padx=8)
    ttk.Spinbox(controls, from_=20, to=150, increment=2, textvariable=gap, width=6).pack(side='left')
    map_point = tk.BooleanVar(value=False)
    ttk.Checkbutton(controls, text='Nhấp = điểm bản đồ', variable=map_point).pack(side='left', padx=8)
    persistent = tk.BooleanVar(value=persistent_map)
    ttk.Checkbutton(controls, text='Bản đồ lâu dài', variable=persistent).pack(side='left')
    running = [False]
    sim = [Simulation(scenario=initial_scenario, config=config, persistent_map=persistent_map)]
    status = tk.StringVar()
    canvases = []
    panels = ttk.Frame(root)
    panels.pack(fill='both', expand=True)
    for title in ('Thế giới giả lập — nhấp để đặt mục tiêu', 'Bản đồ robot quan sát + đường đi'):
        panel = ttk.Frame(panels)
        panel.pack(side='left', fill='both', expand=True)
        ttk.Label(panel, text=title).pack()
        canvas = tk.Canvas(panel, bg='#101827', highlightthickness=0)
        canvas.pack(fill='both', expand=True)
        canvases.append(canvas)

    def transform(canvas, p):
        scale = min(canvas.winfo_width()/4.8, canvas.winfo_height()/4.8)
        return ((p[0]+1.2)*scale, canvas.winfo_height()/2-p[1]*scale)

    def draw():
        model = sim[0]
        for index, canvas in enumerate(canvases):
            canvas.delete('all')
            scale = min(canvas.winfo_width()/4.8, canvas.winfo_height()/4.8)
            if index == 0:
                for value in np.arange(-3., 5., .5):
                    canvas.create_line(*transform(canvas, (value, -3)), *transform(canvas, (value, 3)), fill='#263345')
                    canvas.create_line(*transform(canvas, (-2, value)), *transform(canvas, (4, value)), fill='#263345')
                for x0, y0, x1, y1 in model.boxes:
                    canvas.create_rectangle(*transform(canvas, (x0, y1)), *transform(canvas, (x1, y0)), fill='#96754e')
            else:
                grid = model.nav.grid
                for row, col in np.argwhere(grid.raw >= 0):
                    color = ('#13c7dc' if grid.raw[row, col] == 100 else
                             '#c89b32' if grid.inflated[row, col] else
                             '#743d51' if not grid.allowed[row, col] else '#344255')
                    x, y = transform(canvas, grid.point((row, col)))
                    half = grid.resolution*scale/2
                    canvas.create_rectangle(x-half, y-half, x+half, y+half, fill=color, outline='')
            for points, color in ((model.trail, '#7b8796'), (model.nav.path, '#ce8aff')):
                if len(points) > 1:
                    canvas.create_line(*[v for p in points for v in transform(canvas, p)], fill=color, width=2)
            if model.search is None:
                x, y = transform(canvas, model.target)
                canvas.create_oval(x-6, y-6, x+6, y+6, fill='#35df79', outline='')
            else:
                if index == 1:
                    # Cells the camera has looked at (search coverage), lightly marked.
                    half = model.nav.cfg.resolution*scale/2
                    for i, j in list(model.search.covered)[::3]:
                        x, y = transform(canvas, ((i+.5)*model.nav.cfg.resolution, (j+.5)*model.nav.cfg.resolution))
                        canvas.create_rectangle(x-half, y-half, x+half, y+half, outline='', fill='#3f5f4f')
                if model.person is not None and index == 0:
                    x, y = transform(canvas, model.person)
                    canvas.create_oval(x-9, y-9, x+9, y+9, fill='#e879f9', outline='')
                if model.search.goal is not None and model.search.found is None:
                    x, y = transform(canvas, model.search.goal)
                    canvas.create_polygon(x, y-8, x+8, y, x, y+8, x-8, y, outline='#facc15', fill='', width=2)
                if model.search.found is not None:
                    x, y = transform(canvas, model.search.found)
                    canvas.create_oval(x-13, y-13, x+13, y+13, outline='#ef4444', width=4)
                    canvas.create_text(x+16, y-16, text='ĐÃ TÌM THẤY', fill='#ef4444', anchor='w')
            cc = model.nav.cc
            corners = to_world([[-cc.body_length/2, -cc.body_width/2], [cc.body_length/2, -cc.body_width/2],
                                [cc.body_length/2, cc.body_width/2], [-cc.body_length/2, cc.body_width/2]], model.pose)
            canvas.create_polygon(*[v for p in corners for v in transform(canvas, p)], fill='#ff9b39')
            canvas.create_line(*transform(canvas, model.pose[:2]),
                               *transform(canvas, to_world([[.23, 0]], model.pose)[0]), fill='white', width=3)
        rejected = ('Điểm bị từ chối: {} (chưa quan sát hoặc bị chặn, giống robot)\n'.format(model.goal_error)
                    if model.goal_error else '')
        status.set(rejected + 'Mục tiêu: {} | t={:.1f}s | Navigation: {} | Monitor: {}\nLệnh yêu cầu: ({:.3f}, {:.3f}) m/s, quay {:.2f} rad/s → thực thi: ({:.3f}, {:.3f}) m/s, quay {:.2f} rad/s'.format(
            'điểm bản đồ' if model.exact_goal else 'vật (vision)', model.time-10, model.state,
            model.monitor_state, *model.command, *model.checked) +
            '\nBản đồ: nhớ {:.0f} s | Ô đã biết: {} | Ô đã loại: hết hạn {}, ngoài cửa sổ {}'.format(
                model.nav.costmap.memory_s, model.nav.costmap.known_cells,
                model.nav.costmap.expired_cells, model.nav.costmap.window_evicted_cells))
        if model.nav.costmap.global_map is not None:
            status.set(status.get()+' | Bản đồ toàn cục được giữ lâu dài')

    def reset():
        try:
            width = float(gap.get())/100.
            if not .2 <= width <= 1.5:
                raise ValueError()
        except (ValueError, tk.TclError):
            status.set('Khe phải nằm trong khoảng 20–150 cm.')
            return
        running[0] = False
        sim[0].nav.close()
        sim[0] = Simulation(width, scene.get(), config, persistent_map=persistent.get())
        draw()

    def step():
        sim[0].step()
        draw()

    def target(event):
        running[0] = False
        canvas = canvases[0]
        scale = min(canvas.winfo_width()/4.8, canvas.winfo_height()/4.8)
        point = (event.x/scale-1.2, (canvas.winfo_height()/2-event.y)/scale)
        if sim[0].search is not None:
            sim[0].start_search(point)   # search scene: the click moves the person, search restarts
        else:
            sim[0].set_goal(point, map_point.get())
        draw()

    for title, callback in [('Đặt lại cảnh', reset), ('Chạy', lambda: running.__setitem__(0, True)),
                            ('Tạm dừng', lambda: running.__setitem__(0, False)), ('Bước 0,1 s', step)]:
        ttk.Button(controls, text=title, command=callback).pack(side='left', padx=4)
    canvases[0].bind('<Button-1>', target)
    ttk.Label(root, textvariable=status, padding=8).pack(fill='x')
    ttk.Label(root, text='Cyan: vật cản | Vàng: đệm {:.0f} cm | Đỏ: không đủ chỗ cho tâm xe | Tím: đường đi | Lưới trái: 50 cm\n'
              'LiDAR và odometry lý tưởng; cấu hình: {}. Không mô phỏng YOLO, ROS hay độ trễ phần cứng.'.format(
                  config[0].inflation_radius*100, config_name), padding=8).pack(fill='x')

    def tick():
        if running[0]:
            step()
            if sim[0].state in ('ARRIVED', 'STUCK', 'FOUND', 'SEARCH_COMPLETE'):
                running[0] = False
        root.after(100, tick)
    root.after(150, draw)
    root.after(200, tick)
    root.mainloop()
    sim[0].nav.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--scenario', choices=['gap', 'detour', 'room', 'search', 'empty'], default='gap')
    parser.add_argument('--gap', type=float, default=.60, help='Gap width in metres')
    parser.add_argument('--defaults', action='store_true',
                        help='Use Python defaults instead of the robot YAML values')
    parser.add_argument('--persistent-map', action='store_true', help='Use a persistent simulated floor plan')
    args = parser.parse_args()
    config, config_name = (NavConfig(), Config()), 'mặc định trong code'
    if not args.defaults:
        try:
            config, config_name = robot_config(), 'YAML của robot'
        except ImportError:
            print('PyYAML missing (python -m pip install pyyaml); using Python defaults.')
    # ASCII only: a cp1252 Windows console cannot print Vietnamese text.
    print('Config:', 'Python defaults' if args.defaults or config_name.startswith('m') else 'robot YAML')
    if args.headless:
        sim = Simulation(args.gap, args.scenario, config, persistent_map=args.persistent_map)
        for _ in range(600):
            sim.step()
            if sim.state in ('ARRIVED', 'STUCK', 'FOUND', 'SEARCH_COMPLETE'):
                break
        print('Navigation={} Monitor={} position={} simulated_seconds={:.1f}'.format(
            sim.state, sim.monitor_state, sim.pose.tolist(), sim.time-10.))
        sim.nav.close()
    else:
        desktop(config, config_name, args.scenario, args.persistent_map)
