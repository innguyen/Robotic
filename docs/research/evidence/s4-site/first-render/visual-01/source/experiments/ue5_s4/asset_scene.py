"""UE import and composition of hash-locked meshes and texture PBR materials."""
import hashlib
import json
import math
from pathlib import Path
import unreal as ue


def import_obj(source, name, architectural=False):
    path="/Game/SarAssets/"+name
    mesh=ue.load_asset(path)
    if mesh:return mesh
    task=ue.AssetImportTask()
    for key,value in (("filename",str(source)),("destination_path","/Game/SarAssets"),
                      ("destination_name",name),("automated",True),("save",True)):
        task.set_editor_property(key,value)
    options=ue.FbxImportUI()
    for key,value in (("import_mesh",True),("import_as_skeletal",False),
        ("automated_import_should_detect_type",False),("mesh_type_to_import",ue.FBXImportType.FBXIT_STATIC_MESH),
        ("import_materials",False),("import_textures",False)):
        options.set_editor_property(key,value)
    data=options.static_mesh_import_data
    for key,value in (("combine_meshes",True),("convert_scene",False),("convert_scene_unit",False),("auto_generate_collision",not architectural)):
        data.set_editor_property(key,value)
    task.set_editor_property("options",options)
    ue.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    mesh=ue.load_asset(path)
    if mesh is None:raise RuntimeError("asset mesh import failed: "+str(source))
    if architectural:
        # A convex hull around disconnected floors/facades would fill door
        # openings and the occupied courtyard. These fixtures never move.
        body = mesh.get_editor_property("body_setup")
        body.set_editor_property("collision_trace_flag", ue.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
        body.set_editor_property("double_sided_geometry", True)
        ue.EditorAssetLibrary.save_loaded_asset(mesh)
    # Complex rays see the actual chipped surface; sweeps use the imported
    # convex hull. The conservative authoring boxes exclude hull interiors.
    return mesh


def texture_pbr(root, name, lock):
    files={item["file"]:item for item in lock["files"]}
    signature=hashlib.sha256("".join(files[name+"/"+channel+".jpg"]["sha256"] for channel in ("Diffuse","nor_dx","Rough")).encode()).hexdigest()[:12]
    asset_name="M_"+name+"_"+signature
    existing=ue.load_asset("/Game/SarAssets/"+asset_name)
    if existing:return existing
    textures={}
    for channel in ("Diffuse","nor_dx","Rough"):
        relative=name+"/"+channel+".jpg";item=files[relative];source=root/"source"/relative
        if hashlib.sha256(source.read_bytes()).hexdigest()!=item["sha256"]:raise RuntimeError("PBR hash mismatch")
        texture_name="T_"+name+"_"+channel+"_"+item["sha256"][:10]
        task=ue.AssetImportTask()
        for key,value in (("filename",str(source)),("destination_path","/Game/SarAssets"),("destination_name",texture_name),("automated",True),("save",True)):
            task.set_editor_property(key,value)
        ue.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
        texture=ue.load_asset("/Game/SarAssets/"+texture_name)
        if texture is None:raise RuntimeError("PBR import failed")
        texture.set_editor_property("srgb",channel=="Diffuse")
        if channel=="nor_dx":texture.set_editor_property("compression_settings",ue.TextureCompressionSettings.TC_NORMALMAP)
        ue.EditorAssetLibrary.save_loaded_asset(texture)
        textures[channel]=texture
    asset=ue.AssetToolsHelpers.get_asset_tools().create_asset(asset_name,"/Game/SarAssets",ue.Material,ue.MaterialFactoryNew())
    library=ue.MaterialEditingLibrary
    for channel,prop in (("Diffuse",ue.MaterialProperty.MP_BASE_COLOR),("nor_dx",ue.MaterialProperty.MP_NORMAL),("Rough",ue.MaterialProperty.MP_ROUGHNESS)):
        expression=library.create_material_expression(asset,ue.MaterialExpressionTextureSample)
        expression.set_editor_property("texture",textures[channel])
        expression.set_editor_property("sampler_type",ue.MaterialSamplerType.SAMPLERTYPE_NORMAL if channel=="nor_dx" else ue.MaterialSamplerType.SAMPLERTYPE_COLOR if channel=="Diffuse" else ue.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR)
        library.connect_material_property(expression,"R" if channel=="Rough" else "RGB",prop)
    library.recompile_material(asset);ue.EditorAssetLibrary.save_loaded_asset(asset)
    return asset


def sky_material(root, lock):
    relative = "kloofendal_48d_partly_cloudy_puresky/sky.hdr"
    item = next(item for item in lock["files"] if item["file"] == relative)
    source = root/"source"/relative
    if hashlib.sha256(source.read_bytes()).hexdigest() != item["sha256"]:
        raise ValueError("sky capture hash mismatch")
    suffix = item["sha256"][:12]
    name = "M_DaylightCube_v1_"+suffix
    existing = ue.load_asset("/Game/SarAssets/"+name)
    if existing:
        return existing
    texture_name = "T_Daylight_"+suffix
    task = ue.AssetImportTask()
    for key, value in (("filename",str(source)),("destination_path","/Game/SarAssets"),
                       ("destination_name",texture_name),("automated",True),("save",True)):
        task.set_editor_property(key,value)
    ue.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    texture = ue.load_asset("/Game/SarAssets/"+texture_name)
    if not isinstance(texture, ue.TextureCube):
        raise RuntimeError("expected HDR panorama imported as TextureCube")
    asset = ue.AssetToolsHelpers.get_asset_tools().create_asset(name,"/Game/SarAssets",ue.Material,ue.MaterialFactoryNew())
    asset.set_editor_property("shading_model", ue.MaterialShadingModel.MSM_UNLIT)
    asset.set_editor_property("two_sided", True)
    lib = ue.MaterialEditingLibrary
    sample = lib.create_material_expression(asset,ue.MaterialExpressionTextureSampleParameterCube)
    sample.set_editor_property("parameter_name","Daylight")
    sample.set_editor_property("texture",texture)
    sample.set_editor_property("sampler_type",ue.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR)
    camera = lib.create_material_expression(asset,ue.MaterialExpressionCameraVectorWS)
    direction = lib.create_material_expression(asset,ue.MaterialExpressionMultiply)
    direction.set_editor_property("const_b",-1.)
    lib.connect_material_expressions(camera,"",direction,"A")
    lib.connect_material_expressions(direction,"",sample,"Coordinates")
    lib.connect_material_property(sample,"RGB",ue.MaterialProperty.MP_EMISSIVE_COLOR)
    lib.recompile_material(asset)
    ue.EditorAssetLibrary.save_loaded_asset(asset)
    return asset


def wardrobe_materials(root):
    lock = json.loads((root/"lock.json").read_bytes())
    materials = {}
    for slot in ("skin","eyes","bob","shirt","trousers","footwear"):
        filename = slot+"_diffuse.png"
        source = root/"source"/filename
        sha = hashlib.sha256(source.read_bytes()).hexdigest()
        if sha != lock["files"][filename]["sha256"]:
            raise ValueError("wardrobe texture hash mismatch: "+filename)
        name = "M_HM08_v2_"+slot+"_"+sha[:12]
        asset = ue.load_asset("/Game/SarAssets/"+name)
        if asset is None:
            tex_name = "T_HM08_"+slot+"_"+sha[:12]
            task = ue.AssetImportTask()
            for key,value in (("filename",str(source)),("destination_path","/Game/SarAssets"),
                        ("destination_name",tex_name),("automated",True),("save",True)):
                task.set_editor_property(key,value)
            ue.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
            texture = ue.load_asset("/Game/SarAssets/"+tex_name)
            texture.set_editor_property("max_texture_size",1024)
            ue.EditorAssetLibrary.save_loaded_asset(texture)
            asset = ue.AssetToolsHelpers.get_asset_tools().create_asset(name,"/Game/SarAssets",ue.Material,ue.MaterialFactoryNew())
            library = ue.MaterialEditingLibrary
            sample = library.create_material_expression(asset,ue.MaterialExpressionTextureSample)
            sample.set_editor_property("texture",texture)
            library.connect_material_property(sample,"RGB",ue.MaterialProperty.MP_BASE_COLOR)
            if slot == "bob":
                asset.set_editor_property("blend_mode",ue.BlendMode.BLEND_MASKED)
                asset.set_editor_property("two_sided",True)
                library.connect_material_property(sample,"A",ue.MaterialProperty.MP_OPACITY_MASK)
            rough = library.create_material_expression(asset,ue.MaterialExpressionConstant)
            rough.set_editor_property("r",.30 if slot=="eyes" else .82)
            library.connect_material_property(rough,"",ue.MaterialProperty.MP_ROUGHNESS)
            library.recompile_material(asset)
            ue.EditorAssetLibrary.save_loaded_asset(asset)
        materials[slot] = asset
    return materials


def initialize(world, asset_root):
    root=Path(asset_root)/"environment"
    lock=json.loads((root/"lock.json").read_text())
    metadata=json.loads((root/"meshes.json").read_text())
    world.environment_meshes={}
    world.environment_metadata=metadata
    for name,item in metadata.items():
        source=root/"derived"/item["file"]
        if hashlib.sha256(source.read_bytes()).hexdigest()!=item["sha256"]:raise RuntimeError("scene mesh hash mismatch")
        architectural = "site_layer" in item
        prefix = "SM_TriSite_" if architectural else "SM_"
        world.environment_meshes[name]=import_obj(source,prefix+name+"_"+item["sha256"][:12],architectural)
    world.asset_materials={name:texture_pbr(root,name,lock) for name,item in lock["assets"].items() if item["kind"]!="sky"}
    if any(item["kind"]=="sky" for item in lock["assets"].values()):
        from experiments.ue5_s4.human_scene import material
        world.asset_materials.update(
            Daylight=sky_material(root,lock), Window=material("WorkshopGlass",(.10,.16,.18),roughness=.24),
            Paint=material("WornRoadMarking",(.64,.55,.32)), Diffuser=material("LampDiffuser",(.72,.69,.59)))


def build_architecture(world, environment):
    for name, item in world.environment_metadata.items():
        layer = item.get("site_layer")
        if not layer or layer["environment"] != environment:
            continue
        actor = world.actors.spawn_actor_from_class(ue.StaticMeshActor, ue.Vector(0,0,0))
        actor.set_actor_label(name)
        component = actor.static_mesh_component
        component.set_static_mesh(world.environment_meshes[name])
        finish = layer["finish"]
        component.set_material(0,world.asset_materials[finish] if finish in world.asset_materials else world.materials[finish])
        component.set_collision_profile_name("BlockAll")
        world.spawned.append(actor)


def place(world, name, mesh_id, center, size, material, collision=True, yaw_deg=0):
    actor=world.actors.spawn_actor_from_class(ue.StaticMeshActor,ue.Vector(center[0]*100,-center[1]*100,center[2]*100))
    actor.set_actor_label(name)
    component=actor.static_mesh_component
    component.set_mobility(ue.ComponentMobility.MOVABLE)
    component.set_static_mesh(world.environment_meshes[mesh_id])
    low,high=world.environment_metadata[mesh_id]["bounds_ue_m"]
    turn=mesh_id=="fractured_wall" and size[1]>size[0]
    dimensions=[size[1],size[0],size[2]] if turn else size
    scale=[dimensions[i]/(high[i]-low[i]) for i in range(3)]
    actor.set_actor_scale3d(ue.Vector(*scale))
    offset=[(low[i]+high[i])*.5*scale[i] for i in range(3)]
    yaw = 90 if turn else -yaw_deg
    if yaw:
        actor.set_actor_rotation(ue.Rotator(yaw=yaw), False)
        angle = math.radians(yaw)
        x, y, z = offset
        offset = [x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle), z]
    actor.set_actor_location(ue.Vector(center[0]*100-offset[0]*100,-center[1]*100-offset[1]*100,center[2]*100-offset[2]*100),False,False)
    for i in range(component.get_num_materials()):component.set_material(i,material)
    component.set_collision_profile_name("BlockAll")
    if not collision:component.set_collision_enabled(ue.CollisionEnabled.NO_COLLISION)
    world.spawned.append(actor)
    return actor


def build(world, config):
    material={"Concrete":"cracked_concrete","Brick":"broken_brick_wall","Debris":"concrete_debris"}
    place(world,"asset_ground","panel_"+config["environment"]+"_ground",[3,0,-.06],[9,8.4,.12],
          world.asset_materials["cracked_concrete" if config["environment"]=="indoor" else "concrete_debris"])
    if config["scene_variant"].endswith("_v2"):
        build_architecture(world,config["environment"])
    for box in config["boxes"]:
        key=box.get("material","Concrete")
        pbr=world.asset_materials.get(material.get(key,key),world.materials.get(key))
        if "mesh" in box:
            place(world,box["id"],box["mesh"],box["center_m"],box["size_m"],pbr)
        else:
            panel="panel_"+config["environment"]+"_"+box["id"]
            if panel in world.environment_meshes:
                place(world,box["id"],panel,box["center_m"],box["size_m"],pbr)
            else:
                actor=world.shape(box["id"],box["center_m"],box["size_m"],"Concrete")
                actor.static_mesh_component.set_material(0,pbr)
    # Exposed steel rises from the collapsed wall line, away from traversable
    # floor; the bent bars are visual and never add hidden navigation obstacles.
    for index,(x,y) in enumerate(((2.65,1.3),(5.9,3.4)) if config["environment"]=="indoor" else ((1.2,3.5),(4.7,3.5))):
        place(world,"bent_rebar_"+str(index),"exposed_rebar",[x,y,1.6],[.65,.15,1.2],world.materials["Metal"],False)
    if config["environment"]=="indoor":
        for x in (1.4,4.9):world.shape("exposed_ceiling_joist",[x,0,2.65],[.12,7.4,.16],"Metal")
    for index,victim in enumerate(config["victims"]):
        if victim["posture"]=="seated":
            x,y,_=victim["anchor_m"]
            # Kept out of scan/sweep to avoid a hidden hull around the feet.
            yaw = victim.get("yaw_deg", 180)
            heading = math.radians(yaw)
            # The source chair back lies on local -X. Centre its seat under
            # the posed hips and thighs, facing the same way as the torso.
            center = [x+.08*math.cos(heading), y+.08*math.sin(heading), .44]
            place(world, "victim_seat_"+str(index), "painted_wooden_chair_02",
                  center, [.49,.49,.88], world.asset_materials["painted_wooden_chair_02"],
                  collision=False, yaw_deg=yaw)
