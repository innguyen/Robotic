"""Desktop simulation of the production navigation core; no ROS connection."""
import math
import argparse
import numpy as np

from collision_monitor_ros1 import Config, Monitor, scan_points
from navigation_core_ros1 import Navigator, NavConfig, to_world


class Simulation:
    def __init__(self, gap=.60, scenario='gap'):
        self.nav = Navigator(NavConfig(), Config())
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
                      else [[.8, -.16, 1.15, .16]] if scenario == 'detour' else [])
        self.angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        self.observe()

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
        ranges = self.scan()
        self.nav.costmap.update(ranges, -math.pi, 2*math.pi/720, .05, 6., self.pose, self.time)
        self.monitor.set_scan(scan_points(ranges, -math.pi, 2*math.pi/720, .05, 6., self.nav.cc), self.time)
        self.nav.grid = self.nav.costmap.snapshot(self.pose, self.time)

    def step(self):
        self.time += .1
        self.observe()
        self.command, self.state = self.nav.step(self.pose, self.target, .35, self.time, .1)
        self.monitor.set_command(*self.command, now=self.time)
        self.checked, self.monitor_state = self.monitor.evaluate(self.time)
        vx, vy, wz = self.checked
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose += np.array([c*vx-s*vy, s*vx+c*vy, wz])*.1
        self.trail.append(self.pose[:2].copy())


def desktop():
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title('JetAuto — mô phỏng thuật toán ROS 1 (không kết nối robot)')
    root.geometry('1140x740')
    controls = ttk.Frame(root, padding=8)
    controls.pack(fill='x')
    scene = tk.StringVar(value='gap')
    ttk.Label(controls, text='Cảnh:').pack(side='left')
    ttk.Combobox(controls, textvariable=scene, values=['gap', 'detour', 'empty'], width=9,
                 state='readonly').pack(side='left')
    gap = tk.DoubleVar(value=60.)
    ttk.Label(controls, text='Khe (cm):').pack(side='left', padx=8)
    ttk.Spinbox(controls, from_=20, to=150, increment=2, textvariable=gap, width=6).pack(side='left')
    running = [False]
    sim = [Simulation()]
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
            x, y = transform(canvas, model.target)
            canvas.create_oval(x-6, y-6, x+6, y+6, fill='#35df79', outline='')
            cc = model.nav.cc
            corners = to_world([[-cc.body_length/2, -cc.body_width/2], [cc.body_length/2, -cc.body_width/2],
                                [cc.body_length/2, cc.body_width/2], [-cc.body_length/2, cc.body_width/2]], model.pose)
            canvas.create_polygon(*[v for p in corners for v in transform(canvas, p)], fill='#ff9b39')
            canvas.create_line(*transform(canvas, model.pose[:2]),
                               *transform(canvas, to_world([[.23, 0]], model.pose)[0]), fill='white', width=3)
        status.set('t={:.1f}s | Navigation: {} | Monitor: {}\nLệnh yêu cầu: ({:.3f}, {:.3f}) m/s → thực thi: ({:.3f}, {:.3f}) m/s'.format(
            model.time-10, model.state, model.monitor_state, *model.command[:2], *model.checked[:2]))

    def reset():
        try:
            width = float(gap.get())/100.
            if not .2 <= width <= 1.5:
                raise ValueError()
        except (ValueError, tk.TclError):
            status.set('Khe phải nằm trong khoảng 20–150 cm.')
            return
        running[0] = False
        sim[0] = Simulation(width, scene.get())
        draw()

    def step():
        sim[0].step()
        draw()

    def target(event):
        running[0] = False
        canvas = canvases[0]
        scale = min(canvas.winfo_width()/4.8, canvas.winfo_height()/4.8)
        sim[0].target = np.array([event.x/scale-1.2, (canvas.winfo_height()/2-event.y)/scale])
        sim[0].nav.reset()
        draw()

    for title, callback in [('Đặt lại cảnh', reset), ('Chạy', lambda: running.__setitem__(0, True)),
                            ('Tạm dừng', lambda: running.__setitem__(0, False)), ('Bước 0,1 s', step)]:
        ttk.Button(controls, text=title, command=callback).pack(side='left', padx=4)
    canvases[0].bind('<Button-1>', target)
    ttk.Label(root, textvariable=status, padding=8).pack(fill='x')
    ttk.Label(root, text='Cyan: vật cản | Vàng: đệm 6 cm | Đỏ: không đủ chỗ cho tâm xe | Tím: đường đi | Lưới trái: 50 cm\n'
              'LiDAR và odometry lý tưởng; dùng cấu hình mặc định trong code. Không mô phỏng YOLO, ROS hay độ trễ phần cứng.', padding=8).pack(fill='x')

    def tick():
        if running[0]:
            step()
            if sim[0].state in ('ARRIVED', 'STUCK'):
                running[0] = False
        root.after(100, tick)
    root.after(150, draw)
    root.after(200, tick)
    root.mainloop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--scenario', choices=['gap', 'detour', 'empty'], default='gap')
    parser.add_argument('--gap', type=float, default=.60, help='Gap width in metres')
    args = parser.parse_args()
    if args.headless:
        sim = Simulation(args.gap, args.scenario)
        for _ in range(600):
            sim.step()
            if sim.state in ('ARRIVED', 'STUCK'):
                break
        print('Navigation={} Monitor={} position={} simulated_seconds={:.1f}'.format(
            sim.state, sim.monitor_state, sim.pose.tolist(), sim.time-10.))
    else:
        desktop()
