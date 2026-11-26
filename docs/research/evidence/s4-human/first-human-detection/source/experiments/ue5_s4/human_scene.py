"""UE-side original SAR layouts with baked licensed human meshes and lit materials."""
import json
import math
import os
from pathlib import Path
import unreal as ue


def material(name, color, roughness=.8, unlit=False, patterned=False):
    path = "/Game/SarHuman/M_" + name
    existing = ue.load_asset(path)
    if existing: return existing
    asset = ue.AssetToolsHelpers.get_asset_tools().create_asset("M_" + name, "/Game/SarHuman", ue.Material, ue.MaterialFactoryNew())
    library = ue.MaterialEditingLibrary
    constant = library.create_material_expression(asset, ue.MaterialExpressionConstant3Vector)
    constant.set_editor_property("constant", ue.LinearColor(*color, 1))
    output = constant
    if patterned:
        noise = library.create_material_expression(asset, ue.MaterialExpressionNoise)
        noise.set_editor_property("scale", .045)
        noise.set_editor_property("levels", 2)
        noise.set_editor_property("output_min", .45)
        noise.set_editor_property("output_max", 1.)
        multiply = library.create_material_expression(asset, ue.MaterialExpressionMultiply)
        library.connect_material_expressions(constant, "", multiply, "A")
        library.connect_material_expressions(noise, "", multiply, "B")
        output = multiply
    if unlit:
        asset.set_editor_property("shading_model", ue.MaterialShadingModel.MSM_UNLIT)
        asset.set_editor_property("two_sided", True)
    library.connect_material_property(output, "", ue.MaterialProperty.MP_EMISSIVE_COLOR if unlit else ue.MaterialProperty.MP_BASE_COLOR)
    scalar = library.create_material_expression(asset, ue.MaterialExpressionConstant)
    scalar.set_editor_property("r", roughness)
    library.connect_material_property(scalar, "", ue.MaterialProperty.MP_ROUGHNESS)
    library.recompile_material(asset)
    ue.EditorAssetLibrary.save_loaded_asset(asset)
    return asset


def import_humans(asset_root):
    metadata = json.loads((Path(asset_root) / "derived/poses.json").read_text())
    result = {}
    for posture, item in metadata.items():
        path = "/Game/SarHuman/" + posture
        mesh = ue.load_asset(path)
        if mesh is None:
            task = ue.AssetImportTask()
            task.set_editor_property("filename", str(Path(asset_root) / "derived" / item["file"]))
            task.set_editor_property("destination_path", "/Game/SarHuman")
            task.set_editor_property("destination_name", posture)
            task.set_editor_property("automated", True)
            task.set_editor_property("save", True)
            options = ue.FbxImportUI()
            options.set_editor_property("import_mesh", True)
            options.set_editor_property("import_as_skeletal", False)
            options.set_editor_property("automated_import_should_detect_type", False)
            options.set_editor_property("mesh_type_to_import", ue.FBXImportType.FBXIT_STATIC_MESH)
            options.set_editor_property("import_materials", False)
            options.set_editor_property("import_textures", False)
            data = options.static_mesh_import_data
            data.set_editor_property("combine_meshes", True)
            data.set_editor_property("convert_scene", False)
            data.set_editor_property("convert_scene_unit", False)
            data.set_editor_property("auto_generate_collision", True)
            task.set_editor_property("options", options)
            ue.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
            mesh = ue.load_asset(path)
        if mesh is None: raise RuntimeError("human import failed: " + posture)
        result[posture] = mesh
    return result, metadata


class HumanSceneMixin:
    def init_human(self):
        self.width, self.height = [int(v) for v in os.environ.get("SAR_RGB_SIZE", "320x240").split("x")]
        fx = self.width / 2
        self.camera = {"width": self.width, "height": self.height, "fx": fx, "fy": fx,
                       "cx": (self.width-1)/2, "cy": (self.height-1)/2,
                       "T_base_optical": [[0, 0, 1, .25], [-1, 0, 0, 0], [0, -1, 0, .65], [0, 0, 0, 1]]}
        for target in (self.rgb_target, self.depth_target):
            ue.RenderingLibrary.resize_render_target2d(target, self.width, self.height)
        self.human_meshes, self.pose_metadata = import_humans(os.environ["SAR_ASSET_ROOT"])
        self.materials.update({name: material(name, color, patterned=pattern) for name, color, pattern in (
            ("Concrete", (.35, .32, .28), True), ("Brick", (.4, .18, .10), True),
            ("Floor", (.22, .23, .25), True), ("Ground", (.24, .18, .11), True),
            ("Metal", (.12, .15, .18), False), ("Shirt", (.32, .45, .65), False),
            ("Trousers", (.08, .10, .15), False), ("Skin", (.62, .38, .23), False),
            ("Tire", (.025, .025, .025), True))})
        self.materials["Sky"] = material("Sky", (.18, .27, .40), unlit=True)

    def build_human_scene(self, config):
        self.shape("ground", [3, 0, -.1], [20, 16, .1], "Ground" if config["environment"] == "outdoor" else "Floor")
        for box in config["boxes"]:
            self.shape(box["id"], box["center_m"], box["size_m"], box.get("material", "Concrete"))
        if config["environment"] == "indoor":
            self.shape("ceiling_beam", [3, 0, 2.65], [6, .18, .3], "Metal")
        sky = self.shape("sky", [0, 0, 0], [300, 300, 300], "Sky", "Sphere")
        sky.static_mesh_component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
        sun = self.actors.spawn_actor_from_class(ue.DirectionalLight, ue.Vector(0, 0, 500), ue.Rotator(pitch=-45, yaw=-35, roll=0))
        sun.light_component.set_editor_property("intensity", .8 if config["perception"] else 4.)
        self.spawned.append(sun)
        fill = self.actors.spawn_actor_from_class(ue.DirectionalLight, ue.Vector(0, 0, 500), ue.Rotator(pitch=-25, yaw=140, roll=0))
        fill.light_component.set_editor_property("intensity", .3 if config["perception"] else 1.8)
        self.spawned.append(fill)
        self.victim_actors, self.victims = {}, []
        for index, victim in enumerate(config["victims"]):
            actor = self.actors.spawn_actor_from_class(ue.StaticMeshActor, ue.Vector(victim["anchor_m"][0]*100, -victim["anchor_m"][1]*100, victim["anchor_m"][2]*100))
            actor.set_actor_label("human_" + str(index))
            component = actor.static_mesh_component
            component.set_mobility(ue.ComponentMobility.MOVABLE)
            mesh = self.human_meshes[victim["posture"]]
            component.set_static_mesh(mesh)
            component.set_collision_profile_name("BlockAll")
            actor.set_actor_rotation(ue.Rotator(yaw=victim.get("yaw_deg", 180)), False)
            slots = mesh.get_editor_property("static_materials")
            for i, slot in enumerate(slots):
                key = str(slot.material_slot_name).lower()
                component.set_material(i, self.materials["Skin" if "skin" in key else "Trousers" if "trouser" in key else "Shirt"])
            self.spawned.append(actor)
            self.victims.append({"gt_id": victim["gt_id"], "torso": actor})
            self.victim_actors[actor.get_name()] = victim["gt_id"]

    def robot_visual(self):
        # Child meshes follow the swept chassis. Rollers are visual only.
        for index, (x, y) in enumerate(((-.13, -.19), (-.13, .19), (.13, -.19), (.13, .19))):
            wheel = self.shape("wheel_" + str(index), [0, 0, .12], [.16, .08, .16], "Tire", "Cylinder")
            wheel.static_mesh_component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
            wheel.attach_to_actor(self.robot, "", ue.AttachmentRule.KEEP_RELATIVE, ue.AttachmentRule.KEEP_RELATIVE, ue.AttachmentRule.KEEP_RELATIVE, False)
            wheel.set_actor_relative_location(ue.Vector(x*100, y*100, -8), False, False)
        mast = self.shape("sensor_mast", [0, 0, .4], [.04, .04, .45], "Metal")
        mast.static_mesh_component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
        mast.attach_to_actor(self.robot, "", ue.AttachmentRule.KEEP_RELATIVE, ue.AttachmentRule.KEEP_RELATIVE, ue.AttachmentRule.KEEP_RELATIVE, False)
        mast.set_actor_relative_location(ue.Vector(0, 0, 23), False, False)
