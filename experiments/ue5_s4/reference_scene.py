"""UE-side import of the frozen reference-site assembly."""
import hashlib
import json
from pathlib import Path

import unreal as ue

from .asset_scene import import_obj, texture_pbr, wardrobe_materials


def debris_material(root, lock, environment):
    """Fade dusty scan albedo into its floor finish over the low skirt of each pile."""
    files = {f["file"]:f["sha256"] for f in lock["files"]}
    floor = "linoleum_brown" if environment=="indoor" else "road_damaged_2"
    inputs = [(floor,"Diffuse"),("rubble","Diffuse"),("rubble","nor_dx"),("rubble","Rough")]
    signature = hashlib.sha256("".join(files[f"{s}/{ch}.jpg"] for s,ch in inputs).encode()).hexdigest()[:12]
    name = "M_BreachSkirt_v1_"+environment+"_"+signature
    existing = ue.load_asset("/Game/SarAssets/"+name)
    if existing:
        return existing
    for s in (floor,"rubble"):
        texture_pbr(root,s,lock)
    mat = ue.AssetToolsHelpers.get_asset_tools().create_asset(name,"/Game/SarAssets",ue.Material,ue.MaterialFactoryNew())
    lib = ue.MaterialEditingLibrary
    def expression(kind):
        return lib.create_material_expression(mat,kind)
    def wire(node,out,target,pin):
        if not lib.connect_material_expressions(node,out,target,pin):
            raise ValueError("unconnected debris shader input: "+pin)
    samples = []
    for s,ch in inputs:
        sample = expression(ue.MaterialExpressionTextureSample)
        texture = ue.load_asset(f"/Game/SarAssets/T_{s}_{ch}_"+files[f"{s}/{ch}.jpg"][:10])
        sample.set_editor_property("texture",texture)
        typ = {"Diffuse":ue.MaterialSamplerType.SAMPLERTYPE_COLOR,"nor_dx":ue.MaterialSamplerType.SAMPLERTYPE_NORMAL,
               "Rough":ue.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR}
        sample.set_editor_property("sampler_type",typ[ch])
        samples.append(sample)
    position = expression(ue.MaterialExpressionWorldPosition)
    mask = expression(ue.MaterialExpressionComponentMask)
    mask.set_editor_property("b",True)
    mask.set_editor_property("r",False)
    mask.set_editor_property("g",False)
    wire(position,"",mask,"")
    amount = expression(ue.MaterialExpressionMultiply)
    amount.set_editor_property("const_b",.052)
    wire(mask,"",amount,"A")
    limit = expression(ue.MaterialExpressionSaturate)
    wire(amount,"",limit,"")
    blend = expression(ue.MaterialExpressionLinearInterpolate)
    wire(samples[0],"RGB",blend,"A")
    wire(samples[1],"RGB",blend,"B")
    wire(limit,"",blend,"Alpha")
    for node,output,prop in ((blend,"",ue.MaterialProperty.MP_BASE_COLOR),(samples[2],"RGB",ue.MaterialProperty.MP_NORMAL),
                             (samples[3],"R",ue.MaterialProperty.MP_ROUGHNESS)):
        if not lib.connect_material_property(node,output,prop):
            raise ValueError("unconnected debris material")
    lib.recompile_material(mat)
    ue.EditorAssetLibrary.save_loaded_asset(mat)
    return mat


def initialize(world, asset_root):
    root = Path(asset_root)/"environment45"
    lock = json.loads((root/"lock.json").read_bytes())
    records = json.loads((root/"meshes.json").read_bytes())
    files = {item["file"]:item for item in lock["files"]}
    world.reference_parts = []
    world.reference_wardrobe = {}
    for env, colour in (("indoor","teal"),("outdoor","tan")):
        world.reference_wardrobe[env] = wardrobe_materials(
            root/"source"/"wardrobe", override={"shirt":"shirt_"+colour+".png","trousers":"trousers_blue.png"},
            files={Path(k).name:v for k,v in files.items() if k.startswith("wardrobe/")})
    surfaces = {s for asset in lock["assets"].values() for s in asset["surfaces"]}
    materials = dict(world.asset_materials)
    for name,item in records.items():
        source = root/"derived"/item["file"]
        sha = hashlib.sha256(source.read_bytes()).hexdigest()
        if sha != item["sha256"]:
            raise ValueError("reference mesh hash mismatch: "+name)
        mesh = import_obj(source,"SM_Ref_"+name+"_"+sha[:12],architectural=True)
        bounds = mesh.get_bounds()
        for axis,lo,hi in zip("xyz",*item["bounds_ue_m"]):
            mid = getattr(bounds.origin,axis)
            radius = getattr(bounds.box_extent,axis)
            if max(abs(mid-radius-100*lo),abs(mid+radius-100*hi)) > .2:
                raise ValueError(f"reference mesh {name} {axis}: native {(mid-radius,mid+radius)} cm, authored {(100*lo,100*hi)} cm")
        slots = []
        for slot in mesh.get_editor_property("static_materials"):
            key = str(slot.material_slot_name)
            finish = item["slots"][key] if "slots" in item else item["finish"]
            if finish in surfaces and finish not in materials:
                materials[finish] = texture_pbr(root,finish,lock,two_sided="facade" in finish)
            material = materials.get(finish)
            if finish=="rubble":
                material = debris_material(root,lock,item["instances"][0]["environment"])
            slots.append((finish,material))
        if "slots" in item and len(slots) != len(item["slots"]):
            raise ValueError("missing imported material slots: "+name)
        world.reference_parts.append((name,mesh,slots,item["instances"]))
    world.asset_materials["road_damaged_2"] = materials["road_damaged_2"]


def build(world, environment):
    for name,mesh,slots,instances in world.reference_parts:
        for index,placement in enumerate(instances):
            if placement["environment"] != environment:
                continue
            x,y,z = placement["origin_m"]
            rotation = ue.Rotator(yaw=-placement["yaw_deg"],pitch=placement["pitch_deg"],roll=placement["roll_deg"])
            actor = world.actors.spawn_actor_from_class(ue.StaticMeshActor,ue.Vector(100*x,-100*y,100*z),rotation)
            actor.set_actor_label(f"{name}_{index}")
            component = actor.static_mesh_component
            component.set_static_mesh(mesh)
            actor.set_actor_scale3d(ue.Vector(*placement["scale"]))
            for i,(finish,material) in enumerate(slots):
                component.set_material(i,material or world.materials[finish])
            component.set_collision_profile_name("BlockAll")
            world.spawned.append(actor)
    # Enamel wayfinding and dock identifiers belong to the building. They never
    # encode target locations, route advice, or episode-dependent information.
    signs = ([("RECORDS",2.443,-.51,1.76,8),("SERVICE",8.36,-.55,2.74,11)]
             if environment=="indoor" else
             [("01",8.54,-5.7,3.12,25),("02",8.54,.3,3.12,25),("03",8.54,6.3,3.12,25)])
    for label,x,y,z,size in signs:
        actor = world.actors.spawn_actor_from_class(ue.TextRenderActor,ue.Vector(x*100,-y*100,z*100),
                                                   ue.Rotator(yaw=180))
        component = actor.get_component_by_class(ue.TextRenderComponent)
        component.set_text(label)
        component.set_world_size(size)
        component.set_horizontal_alignment(ue.HorizTextAligment.EHTA_CENTER)
        component.set_vertical_alignment(ue.VerticalTextAligment.EVRTA_TEXT_CENTER)
        component.set_text_render_color(ue.Color(228,222,192,255))
        actor.set_actor_label("sign_"+label)
        world.spawned.append(actor)


def survey_routes(world):
    """Private fixed-yaw chassis sweeps in the native collision world, with a blocked control."""
    environment = world.config["environment"]
    if environment=="outdoor":
        routes = {
            "north_aisle":[(0,0),(1.0,1.15),(1.5,1.77),(2.87,1.77),(3.22,1.50),(3.65,.96),(4.23,.42)],
            "south_aisle":[(0,0),(1.27,-.70),(1.49,-1.99),(3.81,-2.00),(4.23,-1.25),(4.23,.42)],
            "through_barrier":[(0,0),(3.16,.50)]}
    else:
        routes = {"records_corridor":[(0,0),(1.31,-1.37),(3.48,-1.39),(4.14,-.81)],
                  "through_partition":[(0,0),(2.93,.38)]}
    result = {}
    ignore = [world.robot]+[a for a in world.spawned if a.get_attach_parent_actor()==world.robot]
    for name,points in routes.items():
        hits = []
        for i,(a,b) in enumerate(zip(points,points[1:])):
            start,end = [ue.Vector(x*100,-y*100,20) for x,y in (a,b)]
            hit = ue.SystemLibrary.box_trace_single(world.world,start,end,ue.Vector(20,20,15),ue.Rotator(),
                  ue.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,ue.DrawDebugTrace.NONE,True)
            if hit:
                values = hit.to_tuple()
                hits.append(dict(segment=i,actor=values[9].get_actor_label(),distance_m=values[3]/100))
        result[name] = dict(waypoints_m=points,hits=hits,expected_clear=not name.startswith("through_"))
    return dict(environment=environment,epoch=world.epoch,routes=result,
                passed=all(row["expected_clear"]== (not row["hits"]) for row in result.values()),
                scope="Private 40 x 40 x 30 cm axis-aligned swept chassis, centre Z 20 cm. Static translation only.")
