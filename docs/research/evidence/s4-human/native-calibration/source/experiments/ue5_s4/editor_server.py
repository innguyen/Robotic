"""One UE5 native host, using the already installed PythonScript plugin.

All UE calls run on the editor thread. The world is deliberately static. Host
transactions advance a swept kinematic proxy; they do not advance Chaos time.
The listener is privileged, token-protected and loopback-only, not an agent API.
"""
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import time
import traceback
import zlib

import unreal as ue

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from sar_benchmark.contracts import canonical, fields, require, strict_loads, vector
from sar_benchmark.geometry import canonical_to_ue
from sar_benchmark.scenario_contract import validate_scenario
from experiments.webots_s2.geometry import footprint_separation


def wire_payload(response):
    return b"Z"+zlib.compress(canonical(response),1)


class World:
    def __init__(self):
        self.actors = ue.get_editor_subsystem(ue.EditorActorSubsystem)
        self.world = ue.get_editor_subsystem(ue.UnrealEditorSubsystem).get_editor_world()
        self.spawned = []
        self.epoch = self.tick = 0
        self.path = 0.0
        self.events = []
        self.config = None
        self.human = os.environ.get("SAR_PROFILE") == "human"
        self.width, self.height = 80, 60
        self.camera = {"width": 80, "height": 60, "fx": 40.0, "fy": 40.0, "cx": 39.5, "cy": 29.5,
                       "T_base_optical": [[0, 0, 1, .25], [-1, 0, 0, 0], [0, -1, 0, .65], [0, 0, 0, 1]]}
        self.rgb_actor, self.rgb_sensor, self.rgb_target = self.capture(ue.SceneCaptureSource.SCS_FINAL_COLOR_LDR, ue.TextureRenderTargetFormat.RTF_RGBA8)
        self.depth_actor, self.depth_sensor, self.depth_target = self.capture(ue.SceneCaptureSource.SCS_SCENE_DEPTH, ue.TextureRenderTargetFormat.RTF_RGBA16F)
        self.materials = {}
        for name, color in (("Victim", (1.0, .01, .01)), ("Prop", (.2, .25, .3)), ("Robot", (.02, .1, .4))):
            path = "/Game/Sar/M_" + name
            material = ue.load_asset(path)
            if material is None:
                material = ue.AssetToolsHelpers.get_asset_tools().create_asset("M_" + name, "/Game/Sar", ue.Material, ue.MaterialFactoryNew())
                expression = ue.MaterialEditingLibrary.create_material_expression(material, ue.MaterialExpressionConstant3Vector)
                expression.set_editor_property("constant", ue.LinearColor(*color, 1.0))
                material.set_editor_property("shading_model", ue.MaterialShadingModel.MSM_UNLIT)
                ue.MaterialEditingLibrary.connect_material_property(expression, "", ue.MaterialProperty.MP_EMISSIVE_COLOR)
                ue.MaterialEditingLibrary.recompile_material(material)
                ue.EditorAssetLibrary.save_loaded_asset(material)
            self.materials[name] = material
        if self.human:
            from experiments.ue5_s4.human_scene import HumanSceneMixin
            self.human_builder = HumanSceneMixin
            HumanSceneMixin.init_human(self)

    def capture(self, source, fmt):
        actor = self.actors.spawn_actor_from_class(ue.SceneCapture2D, ue.Vector(25, 0, 65))
        sensor = actor.capture_component2d
        sensor.set_editor_property("capture_every_frame", False)
        sensor.set_editor_property("capture_on_movement", False)
        sensor.set_editor_property("capture_source", source)
        sensor.set_editor_property("fov_angle", 90.0)
        target = ue.RenderingLibrary.create_render_target2d(self.world, self.width, self.height, fmt)
        sensor.set_editor_property("texture_target", target)
        return actor, sensor, target

    def shape(self, name, center, size, material="Prop", mesh="Cube"):
        actor = self.actors.spawn_actor_from_class(ue.StaticMeshActor, ue.Vector(*canonical_to_ue(center)))
        actor.set_actor_label(name)
        component = actor.static_mesh_component
        component.set_mobility(ue.ComponentMobility.MOVABLE)
        component.set_static_mesh(ue.load_asset(f"/Engine/BasicShapes/{mesh}.{mesh}"))
        actor.set_actor_scale3d(ue.Vector(*size))
        component.set_material(0, self.materials[material])
        component.set_collision_profile_name("BlockAll")
        self.spawned.append(actor)
        return actor

    def reset(self, config):
        validate_scenario(config)
        require(vector(config.get("start_m"), 3), "invalid start")
        require(len(config.get("victims", [])) <= 3, "too many victims")
        for actor in self.spawned:
            self.actors.destroy_actor(actor)
        self.spawned.clear()
        self.epoch += 1
        self.tick, self.path, self.events = 0, 0.0, []
        self.config = config
        self.pose = [config["start_m"][0], config["start_m"][1], config["start_yaw"]]
        self.victim_actors = {}
        self.victims = []
        if self.human:
            self.human_builder.build_human_scene(self, config)
        else:
            self.shape("floor", [3, 0, -.1], [14, 12, .1])
            for item in config["boxes"]:
                self.shape(item["id"], item["center_m"], item["size_m"])
        for index, victim in enumerate([] if self.human else config["victims"]):
            p = victim["anchor_m"]
            torso = self.shape(f"v{index}_torso", p, [.3, .4, .55], "Victim")
            self.victims.append({"gt_id": victim["gt_id"], "torso": torso})
            parts = [torso,
                     self.shape(f"v{index}_head", [p[0], p[1], p[2] + .43], [.25, .25, .25], "Victim", "Sphere"),
                     self.shape(f"v{index}_legs", [p[0] + .1, p[1], p[2] - .4], [.5, .32, .2], "Victim")]
            for part in parts:
                self.victim_actors[part.get_name()] = victim["gt_id"]
        self.robot = self.shape("ugv_ideal_proxy", [self.pose[0], self.pose[1], .2], [.4, .4, .3], "Robot")
        self.robot.set_actor_rotation(ue.Rotator(pitch=0, yaw=-math.degrees(self.pose[2]), roll=0), False)
        if self.human:
            self.human_builder.robot_visual(self)
        self._geometry_check(reset=True)
        self.last_motion = {"requested_body_twist": [0.0] * 3, "actual_world_delta_per_s": [0.0] * 3}

    def _geometry_check(self, reset=False):
        # Reuse the independently tested S2 SAT helper only for assertion, not
        # sensing or target selection. Engine sweeps remain the motion mechanism.
        for box in self.config["boxes"]:
            if box["center_m"][2] - box["size_m"][2] / 2 >= .35:
                continue
            fixture = box["center_m"][:2] + box["size_m"][:2]
            if footprint_separation(self.pose[:2], self.pose[2], fixture) < -1e-4:
                if reset:
                    raise RuntimeError("reset penetration")
                self.events.append("GEOMETRY_PENETRATION")
        if not (-1 <= self.pose[0] <= 7 and abs(self.pose[1]) <= 4.5):
            self.events.append("OUT_OF_BOUNDS")

    def step(self, action):
        require(vector(action, 3) and -.1 <= action[0] <= .3 and abs(action[1]) <= .2 and abs(action[2]) <= .6, "invalid host twist")
        require(self.config is not None, "host not reset")
        start = list(self.pose)
        for _ in range(10):
            x, y, yaw = self.pose
            dx = (math.cos(yaw) * action[0] - math.sin(yaw) * action[1]) * .01
            dy = (math.sin(yaw) * action[0] + math.cos(yaw) * action[1]) * .01
            self.robot.set_actor_location(ue.Vector(*canonical_to_ue([x + dx, y + dy, .2])), True, False)
            actual = self.robot.get_actor_location()
            self.pose = [actual.x / 100, -actual.y / 100, yaw + action[2] * .01]
            self.path += math.hypot(self.pose[0] - x, self.pose[1] - y)
            if math.hypot(self.pose[0] - (x + dx), self.pose[1] - (y + dy)) > 1e-4:
                self.events.append("CONTACT")
            self.robot.set_actor_rotation(ue.Rotator(pitch=0, yaw=-math.degrees(self.pose[2]), roll=0), False)
            self._geometry_check()
            if self.events:
                break
        self.tick += 1
        measured = [(self.pose[i] - start[i]) / .1 for i in range(3)]
        # Displacement is world-frame; retain requested body twist separately.
        self.last_motion = {"requested_body_twist": action, "actual_world_delta_per_s": measured}

    def trace(self, origin, end):
        return ue.SystemLibrary.line_trace_single(self.world, ue.Vector(*canonical_to_ue(origin)),
            ue.Vector(*canonical_to_ue(end)), ue.TraceTypeQuery.TRACE_TYPE_QUERY1, self.human,
            [self.robot], ue.DrawDebugTrace.NONE, True)

    def close(self):
        for actor in self.spawned + [self.rgb_actor, self.depth_actor]:
            self.actors.destroy_actor(actor)
        self.spawned.clear()
        self.victims = []
        self.victim_actors = {}
        self.robot = self.rgb_actor = self.depth_actor = None
        ue.RenderingLibrary.release_render_target2d(self.rgb_target)
        ue.RenderingLibrary.release_render_target2d(self.depth_target)
        self.rgb_target = self.depth_target = self.rgb_sensor = self.depth_sensor = None
        self.materials.clear()
        if self.human:
            self.actors.destroy_actor(self.mask_actor)
            ue.RenderingLibrary.release_render_target2d(self.mask_target)
            self.mask_actor = self.mask_target = self.mask_sensor = None

    def observe(self):
        require(self.config is not None, "host not reset")
        if self.human:
            return self.human_builder.encoded_capture(self)
        actual = self.robot.get_actor_location()
        actual_rotation = self.robot.get_actor_rotation()
        self.pose = [actual.x / 100, -actual.y / 100, -math.radians(actual_rotation.yaw)]
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        origin = [x + .25 * c, y + .25 * s, .65]
        capture_start = time.monotonic_ns()
        for actor, sensor in ((self.rgb_actor, self.rgb_sensor), (self.depth_actor, self.depth_sensor)):
            actor.set_actor_location(ue.Vector(*canonical_to_ue(origin)), False, False)
            actor.set_actor_rotation(ue.Rotator(pitch=0, yaw=-math.degrees(yaw), roll=0), False)
            sensor.capture_scene()
        rgb_colors = ue.RenderingLibrary.read_render_target(self.world, self.rgb_target, False)
        depth_colors = ue.RenderingLibrary.read_render_target_raw(self.world, self.depth_target, False)
        require(rgb_colors is not None and depth_colors is not None and len(rgb_colors) == self.width * self.height and len(depth_colors) == self.width * self.height, "readback failed")
        rgb = [channel for color in rgb_colors for channel in (color.r, color.g, color.b)]
        depth = [color.r / 100 if math.isfinite(color.r) and .01 <= color.r / 100 <= 10 else None for color in depth_colors]
        capture_end = time.monotonic_ns()
        visible = {}
        candidates = () if self.human else ((row,col) for row in range(self.height) for col in range(self.width))
        # Privileged visibility from actual engine collision rays at image pixel
        # centers. Scan all pixels: independent of the public detector or RGB.
        for row, col in candidates:
                # Skip floor/sky rays with no object-range depth. This uses only
                # native clean depth, which is never provided as a GT annotation.
                if depth[row * self.width + col] is None:
                    continue
                right = (col - self.camera["cx"]) / self.camera["fx"]
                down = (row - self.camera["cy"]) / self.camera["fy"]
                direction = [c + s * right, s - c * right, -down]
                end = [origin[i] + 12 * direction[i] for i in range(3)]
                hit = self.trace(origin, end)
                if hit is None:
                    continue
                values = hit.to_tuple()
                actor = values[9]
                if actor is not None and actor.get_name() in self.victim_actors:
                    gt_id = self.victim_actors[actor.get_name()]
                    item = visible.setdefault(gt_id, {"pixels": 0, "bbox": [col, row, col + 1, row + 1]})
                    item["pixels"] += 1
                    box = item["bbox"]
                    box[:] = [min(box[0], col), min(box[1], row), max(box[2], col + 1), max(box[3], row + 1)]
        if self.human:
            visible = self.human_builder.rendered_visibility(self,origin,yaw,depth)
        scan = []
        ray_count = 360 if self.human else 72
        for index in range(ray_count):
            angle = -math.pi + index * math.tau / ray_count + yaw
            hit = self.trace([x, y, .2], [x + 8 * math.cos(angle), y + 8 * math.sin(angle), .2])
            scan.append(hit.to_tuple()[3] / 100 if hit is not None else None)
        actual_victims = []
        for item in self.victims:
            p = item["torso"].get_actor_location()
            actual_victims.append({"gt_id": item["gt_id"], "anchor_m": [p.x / 100, -p.y / 100, p.z / 100]})
        return {"epoch": self.epoch, "tick": self.tick, "pose": self.pose, "path_m": self.path,
                "events": self.events, "rgb": rgb, "depth_m": depth, "scan_m": scan, "camera": self.camera,
                "visible": visible, "victims": actual_victims,
                "motion": self.last_motion, "capture_wall_ns": time.monotonic_ns(),
                "capture_start_wall_ns": capture_start, "capture_end_wall_ns": capture_end,
                "available_wall_ns": time.monotonic_ns(), "profile": "human" if self.human else "diagnostic",
                "camera_rotation": str(self.rgb_actor.get_actor_rotation()),
                "clock_profile": "static_kinematic_transactions_not_Chaos_lockstep"}


class Server:
    def __init__(self):
        self.token = os.environ.pop("SAR_HOST_TOKEN")
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", int(os.environ["SAR_HOST_PORT"])))
        self.listener.listen(1)
        self.listener.setblocking(False)
        self.client = None
        self.buffer, self.output = bytearray(), bytearray()
        self.sequence = 0
        self.world = World()
        self.last_activity = time.monotonic()
        self.exiting = False
        self.pending = None
        Path(os.environ["SAR_HOST_READY"]).write_text(json.dumps({"status": "READY", "engine_version": ue.SystemLibrary.get_engine_version(),
            "port": self.listener.getsockname()[1], "clock_profile": "static_kinematic_transactions_not_Chaos_lockstep",
            "sensors": ["native_RGB8", "native_scene_depth_Z_half_float_cm", "engine_collision_ray_scan"],
            "security": "trusted_local_agents_only_no_OS_sandbox"}), encoding="utf-8")

    def tick(self, delta):
        try:
            if self.pending is not None:
                response, frames = self.pending
                if frames:
                    self.pending = response, frames - 1
                    return
                try:
                    response["result"] = self.world.observe()
                except Exception:
                    response["error"] = traceback.format_exc()
                    ue.log_error(response["error"])
                payload = wire_payload(response)
                self.output.extend(struct.pack("!I", len(payload)) + payload)
                self.pending = None
                return
            if self.exiting and not self.output:
                self.world.close()
                self.listener.close()
                if self.client:
                    self.client.close()
                ue.unregister_slate_post_tick_callback(self.handle)
                ue.EditorPythonScripting.set_keep_python_script_alive(False)
                return
            if time.monotonic() - self.last_activity > 120:
                self.exiting = True
                return
            if self.client is None:
                try:
                    self.client, _ = self.listener.accept()
                    self.client.setblocking(False)
                except BlockingIOError:
                    return
            if self.output:
                try:
                    sent = self.client.send(self.output)
                    del self.output[:sent]
                except BlockingIOError:
                    pass
                return
            try:
                data = self.client.recv(65536)
                if not data:
                    self.exiting = True
                    return
                self.buffer.extend(data)
            except BlockingIOError:
                return
            if len(self.buffer) < 4:
                return
            size = struct.unpack("!I", self.buffer[:4])[0]
            require(0 < size <= 16 * 1024 * 1024, "oversized host request")
            if len(self.buffer) < size + 4:
                return
            request = strict_loads(self.buffer[4:4 + size])
            del self.buffer[:4 + size]
            self.last_activity = time.monotonic()
            fields(request, ("token", "sequence", "operation", "arguments"))
            require(request["token"] == self.token and request["sequence"] == self.sequence, "unauthorized/out-of-order host request")
            response = {"sequence": self.sequence}
            self.sequence += 1
            try:
                op, args = request["operation"], request["arguments"]
                if op == "hello":
                    result = {"engine_version": ue.SystemLibrary.get_engine_version(), "rendered": True, "clock_profile": "static_kinematic_transactions_not_Chaos_lockstep"}
                elif op == "reset":
                    self.world.reset(args)
                    self.pending = response, 3
                    return
                elif op == "step":
                    self.world.step(args["twist"])
                    self.pending = response, 2
                    return
                elif op == "stop":
                    result = {"stopped": True, "tick": self.world.tick}
                elif op == "shutdown":
                    self.exiting, result = True, {"closing": True}
                else:
                    raise ValueError("host operation not allowed")
                response["result"] = result
            except Exception:
                response["error"] = traceback.format_exc()
                ue.log_error(response["error"])
            payload = wire_payload(response)
            self.output.extend(struct.pack("!I", len(payload)) + payload)
        except Exception:
            ue.log_error(traceback.format_exc())
            self.exiting = True


try:
    ue.EditorPythonScripting.set_keep_python_script_alive(True)
    server = Server()
    server.handle = ue.register_slate_post_tick_callback(server.tick)
except Exception:
    Path(os.environ["SAR_HOST_READY"]).write_text(json.dumps({"status": "ERROR", "traceback": traceback.format_exc()}), encoding="utf-8")
    ue.SystemLibrary.quit_editor()
