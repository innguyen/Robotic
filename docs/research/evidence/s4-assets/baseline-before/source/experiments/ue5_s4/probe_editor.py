"""Small real-RHI probe; launched by the installed UE Python plugin."""
import json
import os
from pathlib import Path
import traceback
import unreal as ue

out = Path(os.environ["SAR_PROBE_OUT"])
try:
    world = ue.get_editor_subsystem(ue.UnrealEditorSubsystem).get_editor_world()
    actor = ue.get_editor_subsystem(ue.EditorActorSubsystem).spawn_actor_from_class(
        ue.StaticMeshActor, ue.Vector(300, 0, 65))
    component = actor.static_mesh_component
    component.set_static_mesh(ue.load_asset("/Engine/BasicShapes/Cube.Cube"))
    actor.set_actor_scale3d(ue.Vector(.3, .4, .55))
    capture = ue.get_editor_subsystem(ue.EditorActorSubsystem).spawn_actor_from_class(
        ue.SceneCapture2D, ue.Vector(25, 0, 65), ue.Rotator(0, 0, 0))
    sensor = capture.capture_component2d
    sensor.set_editor_property("capture_every_frame", False)
    sensor.set_editor_property("capture_on_movement", False)
    target = ue.RenderingLibrary.create_render_target2d(world, 80, 60, ue.TextureRenderTargetFormat.RTF_RGBA16F)
    sensor.set_editor_property("texture_target", target)
    sensor.set_editor_property("capture_source", ue.SceneCaptureSource.SCS_SCENE_DEPTH)
    sensor.set_editor_property("fov_angle", 90.0)
    sensor.capture_scene()
    samples = ue.RenderingLibrary.read_render_target_raw(world, target, False)
    result = {"status": "PROBE_ONLY", "version": ue.SystemLibrary.get_engine_version(),
              "samples": len(samples), "depth_center_raw": samples[30 * 80 + 40].r,
              "read_raw_doc": ue.RenderingLibrary.read_render_target_raw.__doc__,
              "trace_doc": ue.SystemLibrary.line_trace_single.__doc__,
              "break_doc": ue.GameplayStatics.break_hit_result.__doc__ if hasattr(ue.GameplayStatics, "break_hit_result") else "use SystemLibrary",
              "sweep_doc": actor.set_actor_location.__doc__}
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
except Exception:
    out.write_text(json.dumps({"status": "ERROR", "traceback": traceback.format_exc()}), encoding="utf-8")
finally:
    ue.SystemLibrary.quit_editor()
