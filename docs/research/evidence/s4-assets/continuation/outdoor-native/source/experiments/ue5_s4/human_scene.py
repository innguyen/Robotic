"""UE-side original SAR layouts with baked licensed human meshes and lit materials."""
import json
import math
import os
from pathlib import Path
import unreal as ue
import base64
import time
import hashlib


def export_target(target,path,fmt):
    path = Path(path)
    path.unlink(missing_ok=True)
    options = ue.ImageWriteOptions()
    options.set_editor_property("format",fmt)
    options.set_editor_property("async_",False)
    options.set_editor_property("overwrite_file",True)
    ue.ImageWriteBlueprintLibrary.export_to_disk(target,str(path),options)
    if not path.is_file(): raise RuntimeError("render export failed: "+path.name)
    return base64.b64encode(path.read_bytes()).decode()


def material(name, color, roughness=.8, unlit=False, patterned=False):
    recipe = hashlib.sha256(repr((color,roughness,unlit,patterned,"pbr-noise-v1")).encode()).hexdigest()[:8]
    asset_name = "M_"+name+"_"+recipe
    path = "/Game/SarHuman/" + asset_name
    existing = ue.load_asset(path)
    if existing: return existing
    asset = ue.AssetToolsHelpers.get_asset_tools().create_asset(asset_name, "/Game/SarHuman", ue.Material, ue.MaterialFactoryNew())
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
        source = Path(asset_root)/"derived"/item["file"]
        if hashlib.sha256(source.read_bytes()).hexdigest() != item["sha256"]:
            raise RuntimeError("derived human hash mismatch")
        asset_name = "H_"+posture+"_"+item["sha256"][:8]
        path = "/Game/SarHuman/" + asset_name
        mesh = ue.load_asset(path)
        if mesh is None:
            task = ue.AssetImportTask()
            task.set_editor_property("filename", str(Path(asset_root) / "derived" / item["file"]))
            task.set_editor_property("destination_path", "/Game/SarHuman")
            task.set_editor_property("destination_name", asset_name)
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
        self.scan_rays = int(os.environ.get("SAR_SCAN_RAYS","360"))
        self.width, self.height = [int(v) for v in os.environ.get("SAR_RGB_SIZE", "320x240").split("x")]
        if self.scan_rays not in (360,720) or (self.width,self.height) not in ((160,120),(320,240),(640,480)):
            raise ValueError("unsupported native sensor profile")
        fx = self.width / 2
        self.camera = {"width": self.width, "height": self.height, "fx": fx, "fy": fx,
                       "cx": (self.width-1)/2, "cy": (self.height-1)/2,
                       "T_base_optical": [[0, 0, 1, .25], [-1, 0, 0, 0], [0, -1, 0, .65], [0, 0, 0, 1]]}
        for target in (self.rgb_target, self.depth_target):
            ue.RenderingLibrary.resize_render_target2d(target, self.width, self.height)
        self.mask_actor,self.mask_sensor,self.mask_target = self.capture(ue.SceneCaptureSource.SCS_SCENE_DEPTH,ue.TextureRenderTargetFormat.RTF_RGBA16F)
        self.mask_sensor.set_editor_property("primitive_render_mode",ue.SceneCapturePrimitiveRenderMode.PRM_USE_SHOW_ONLY_LIST)
        self.human_meshes, self.pose_metadata = import_humans(os.environ["SAR_ASSET_ROOT"])
        self.anatomy_meshes = None
        if os.environ.get("SAR_SCENE_SET")=="assets":
            self.anatomy_meshes,self.anatomy_metadata=import_humans(str(Path(os.environ["SAR_ASSET_ROOT"])/"anatomy"))
            from experiments.ue5_s4.asset_scene import initialize
            initialize(self,os.environ["SAR_ASSET_ROOT"])
        self.materials.update({name: material(name, color, patterned=pattern) for name, color, pattern in (
            ("Concrete", (.35, .32, .28), True), ("Brick", (.4, .18, .10), True),
            ("Floor", (.22, .23, .25), True), ("Ground", (.24, .18, .11), True),
            ("Metal", (.12, .15, .18), False), ("Shirt", (.32, .45, .65), False),
            ("Trousers", (.08, .10, .15), False), ("Skin", (.62, .38, .23), False),
            ("Tire", (.025, .025, .025), True), ("Boots", (.035,.028,.022), False))})
        self.materials["Sky"] = material("Sky", (.18, .27, .40), unlit=True)
        # Keep render infrastructure registered across episode resets. Reusing
        # light scene IDs avoids changing deferred light accumulation order.
        # Own the disposable Entry world lighting. One sun avoids UE's ambiguous
        # forward-shading directional-light selection; fill is a local area lamp.
        for actor in self.actors.get_all_level_actors():
            if isinstance(actor,(ue.DirectionalLight,ue.SkyLight)):
                self.actors.destroy_actor(actor)
        sun=self.actors.spawn_actor_from_class(ue.DirectionalLight,ue.Vector(0,0,500),ue.Rotator(pitch=-45,yaw=-35,roll=0))
        fill=self.actors.spawn_actor_from_class(ue.RectLight,ue.Vector(250,0,300),ue.Rotator(pitch=-90,yaw=0,roll=0))
        self.human_lights=[sun,fill]
        for light in self.human_lights:light.light_component.set_mobility(ue.ComponentMobility.MOVABLE)
        fill.light_component.set_editor_property("source_width",400.)
        fill.light_component.set_editor_property("source_height",300.)
        fill.light_component.set_editor_property("attenuation_radius",1000.)

    def encoded_capture(self):
        """Native lossless PNG/EXR readback; avoid per-pixel Python reflection."""
        actual,rotation = self.robot.get_actor_location(),self.robot.get_actor_rotation()
        self.pose = [actual.x/100,-actual.y/100,-math.radians(rotation.yaw)]
        x,y,yaw = self.pose; c,s = math.cos(yaw),math.sin(yaw)
        origin = [x+.25*c,y+.25*s,.65]
        folder = Path(os.environ["SAR_CAPTURE_DIR"]); folder.mkdir(exist_ok=True)
        def export(target,name,fmt):
            return export_target(target,folder/name,fmt)
        start = time.monotonic_ns()
        for actor,sensor in ((self.rgb_actor,self.rgb_sensor),(self.depth_actor,self.depth_sensor),(self.mask_actor,self.mask_sensor)):
            actor.set_actor_location(ue.Vector(origin[0]*100,-origin[1]*100,origin[2]*100),False,False)
            actor.set_actor_rotation(ue.Rotator(yaw=-math.degrees(yaw)),False)
        self.rgb_sensor.capture_scene(); self.depth_sensor.capture_scene()
        rgb = export(self.rgb_target,"rgb.png",ue.DesiredImageFormat.PNG)
        depth = export(self.depth_target,"depth.exr",ue.DesiredImageFormat.EXR)
        end = time.monotonic_ns()
        masks = {}
        for victim in self.victims:
            self.mask_sensor.clear_show_only_components()
            self.mask_sensor.show_only_actor_components(victim["torso"])
            self.mask_sensor.capture_scene()
            masks[victim["gt_id"]] = export(self.mask_target,"mask.exr",ue.DesiredImageFormat.EXR)
        scan = []
        for index in range(self.scan_rays):
            angle = -math.pi+index*math.tau/self.scan_rays+yaw
            hit = self.trace([x,y,.2],[x+8*math.cos(angle),y+8*math.sin(angle),.2])
            scan.append(hit.to_tuple()[3]/100 if hit else None)
        victims = []
        for victim in self.victims:
            p = victim["torso"].get_actor_location()
            victims.append({"gt_id":victim["gt_id"],"anchor_m":[p.x/100,-p.y/100,p.z/100]})
        return {"epoch":self.epoch,"tick":self.tick,"pose":self.pose,"path_m":self.path,"events":self.events,
                "rgb_png":rgb,"depth_exr":depth,"isolated_depth_exr":masks,"scan_m":scan,"camera":self.camera,
                "victims":victims,"motion":self.last_motion,"profile":"human","capture_start_wall_ns":start,
                "capture_end_wall_ns":end,"capture_wall_ns":start,"available_wall_ns":time.monotonic_ns(),
                "clock_profile":"static_kinematic_transactions_not_Chaos_lockstep",
                "controlled_light_ids":[light.get_name() for light in self.human_lights],
                "lighting":{"directional_count":sum(isinstance(a,ue.DirectionalLight) for a in self.actors.get_all_level_actors()),
                    "intensities":[light.light_component.get_editor_property("intensity") for light in self.human_lights]},
                "render_loop":{"frames":self.render_frames,"last_delta_s":self.last_render_delta},
                "native_geometry_validation":{"overlap_queries":self.native_overlap_queries,"hits":self.native_overlap_hits[-4:]},
                "asset_diagnostics":[{"name":v["torso"].get_name(),"slots":[str(s.material_slot_name) for s in v["torso"].static_mesh_component.static_mesh.get_editor_property("static_materials")],
                    "materials":[str(v["torso"].static_mesh_component.get_material(i).get_path_name()) for i in range(v["torso"].static_mesh_component.get_num_materials())]} for v in self.victims] if self.tick==0 else []}

    def build_human_scene(self, config):
        assets="scene_variant" in config
        if assets:
            if self.anatomy_meshes is None:raise RuntimeError("asset scene requires SAR_SCENE_SET=assets")
            from experiments.ue5_s4.asset_scene import build
            build(self,config)
        else:
            self.shape("ground", [3, 0, -.05], [20, 16, .1], "Ground" if config["environment"] == "outdoor" else "Floor")
            for box in config["boxes"]:
                self.shape(box["id"], box["center_m"], box["size_m"], box.get("material", "Concrete"))
            if config["environment"] == "indoor":
                self.shape("ceiling_beam", [3, 0, 2.65], [6, .18, .3], "Metal")
        sky = self.shape("sky", [0, 0, 0], [300, 300, 300], "Sky", "Sphere")
        sky.static_mesh_component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
        sky.static_mesh_component.set_cast_shadow(False)
        for light,intensity in zip(self.human_lights,(.5,4.) if config["perception"] else (2.5,24.)):
            light.light_component.set_editor_property("intensity",intensity)
        self.victim_actors, self.victims = {}, []
        for index, victim in enumerate(config["victims"]):
            actor = self.actors.spawn_actor_from_class(ue.StaticMeshActor, ue.Vector(victim["anchor_m"][0]*100, -victim["anchor_m"][1]*100, victim["anchor_m"][2]*100))
            actor.set_actor_label("human_" + str(index))
            component = actor.static_mesh_component
            component.set_mobility(ue.ComponentMobility.MOVABLE)
            mesh = (self.anatomy_meshes if assets else self.human_meshes)[victim["posture"]]
            component.set_static_mesh(mesh)
            component.set_collision_profile_name("BlockAll")
            actor.set_actor_rotation(ue.Rotator(yaw=-victim.get("yaw_deg", 180)), False)
            slots = mesh.get_editor_property("static_materials")
            for i, slot in enumerate(slots):
                key = str(slot.material_slot_name).lower()
                component.set_material(i, self.materials["Skin" if "skin" in key else "Trousers" if "trouser" in key else "Boots" if "boots" in key else "Shirt"])
            self.spawned.append(actor)
            self.victims.append({"gt_id": victim["gt_id"], "torso": actor})
            self.victim_actors[actor.get_name()] = victim["gt_id"]

    def overview(self):
        """Trusted QA view only; restore the agent camera and never deliver it."""
        old_location,old_rotation = self.rgb_actor.get_actor_location(),self.rgb_actor.get_actor_rotation()
        x,y,_ = self.pose
        position,look_at = [-.3,-5.8,5.3],[3.,0.,.7]
        yaw = math.atan2(look_at[1]-position[1],look_at[0]-position[0])
        pitch = math.atan2(look_at[2]-position[2],math.dist(position[:2],look_at[:2]))
        try:
            self.rgb_actor.set_actor_location(ue.Vector(position[0]*100,-position[1]*100,position[2]*100),False,False)
            self.rgb_actor.set_actor_rotation(ue.Rotator(pitch=math.degrees(pitch),yaw=-math.degrees(yaw)),False)
            self.rgb_sensor.capture_scene()
            png = export_target(self.rgb_target,Path(os.environ["SAR_CAPTURE_DIR"])/"overview.png",ue.DesiredImageFormat.PNG)
            visual = []
            for actor in self.spawned:
                if actor.get_actor_label().startswith("wheel_") or actor.get_actor_label()=="sensor_mast":
                    p = actor.get_actor_location()
                    scale = actor.get_actor_scale3d()
                    visual.append({"label":actor.get_actor_label(),"position_m":[p.x/100,-p.y/100,p.z/100],"world_scale":[scale.x,scale.y,scale.z]})
            return {"rgb_png":png,"epoch":self.epoch,"tick":self.tick,"camera_world_m":position,
                    "look_at_world_m":look_at,"size":[self.width,self.height],"robot_pose_m":list(self.pose),"robot_visual_poses":visual,
                    "robot_geometry_recipe":self.robot_geometry_recipe,
                    "scope":"PRIVATE QA overview; not an agent observation or scored evidence"}
        finally:
            self.rgb_actor.set_actor_location(old_location,False,False)
            self.rgb_actor.set_actor_rotation(old_rotation,False)

    def robot_visual(self):
        from sar_benchmark.embodiment import MecanumGeometry
        model = MecanumGeometry()
        x0,y0,yaw = self.pose; c,s = math.cos(yaw),math.sin(yaw)
        length,width,radius = model.half_length_m,model.half_width_m,model.wheel_radius_m
        self.robot_geometry_recipe = {"wheel_radius_m":radius,"half_length_m":length,"half_width_m":width,
                                      "wheel_thickness_m":.06,"mast_center_z_m":.43,"mast_size_m":[.04,.04,.45]}
        # The chassis actor has non-unit scale. KEEP_WORLD preserves each
        # child's metric size/offset; relative metre values would be scaled again.
        # Wheels are visual; roller/contact dynamics remain absent.
        for index, (x, y) in enumerate(((-length,-width),(-length,width),(length,-width),(length,width))):
            wheel = self.shape("wheel_" + str(index), [x0+c*x-s*y,y0+s*x+c*y,radius], [2*radius,2*radius,.06], "Tire", "Cylinder")
            wheel.static_mesh_component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
            wheel.set_actor_rotation(ue.Rotator(yaw=-math.degrees(yaw),roll=90),False)
            wheel.attach_to_actor(self.robot, "", ue.AttachmentRule.KEEP_WORLD, ue.AttachmentRule.KEEP_WORLD, ue.AttachmentRule.KEEP_WORLD, False)
        mast = self.shape("sensor_mast", [x0,y0,.43], [.04,.04,.45], "Metal")
        mast.static_mesh_component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
        mast.set_actor_rotation(ue.Rotator(yaw=-math.degrees(yaw)),False)
        mast.attach_to_actor(self.robot, "", ue.AttachmentRule.KEEP_WORLD, ue.AttachmentRule.KEEP_WORLD, ue.AttachmentRule.KEEP_WORLD, False)
