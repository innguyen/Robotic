"""UE import and composition of hash-locked meshes and texture PBR materials."""
import hashlib
import json
import math
from pathlib import Path
import unreal as ue


def import_obj(source, name):
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
    for key,value in (("combine_meshes",True),("convert_scene",False),("convert_scene_unit",False),("auto_generate_collision",True)):
        data.set_editor_property(key,value)
    task.set_editor_property("options",options)
    ue.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    mesh=ue.load_asset(path)
    if mesh is None:raise RuntimeError("asset mesh import failed: "+str(source))
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


def initialize(world, asset_root):
    root=Path(asset_root)/"environment"
    lock=json.loads((root/"lock.json").read_text())
    metadata=json.loads((root/"meshes.json").read_text())
    world.environment_meshes={}
    world.environment_metadata=metadata
    for name,item in metadata.items():
        source=root/"derived"/item["file"]
        if hashlib.sha256(source.read_bytes()).hexdigest()!=item["sha256"]:raise RuntimeError("scene mesh hash mismatch")
        world.environment_meshes[name]=import_obj(source,"SM_"+name+"_"+item["sha256"][:12])
    world.asset_materials={name:texture_pbr(root,name,lock) for name in lock["assets"]}


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
