"""Restore pinned source art and bake selected glTF kit pieces in physical metres."""
from collections import defaultdict
import argparse
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import cv2

from .bake_environment import Mesh
from .bake_humans import rotation
from .prepare_environment import fetch
from .reference_layouts import assemble

ROOT = Path(__file__).parent / "assets/environment45"


def restore(root):
    lock = json.loads((root/"lock.json").read_bytes())
    for item in lock["files"]:
        source = (root/"source"/item["file"]).resolve()
        if not source.is_relative_to((root/"source").resolve()):
            raise ValueError("invalid asset path")
        if item.get("recipe") == "shirt-normal-dx-v1":
            height = cv2.imread(str(root/"source/wardrobe/shirt_height.png"), cv2.IMREAD_GRAYSCALE)
            dx = cv2.Sobel(height, cv2.CV_32F, 1, 0) / 256.
            dy = cv2.Sobel(height, cv2.CV_32F, 0, 1) / 256.
            normal = np.dstack((np.ones_like(dx),-dy,-dx))
            normal /= np.linalg.norm(normal,axis=2,keepdims=True)
            raw = cv2.imencode(".png",np.rint((normal+1)*127.5).astype(np.uint8))[1].tobytes()
            if source.exists() and source.read_bytes() != raw:
                raise ValueError("derived normal differs from source height")
        else:
            raw = source.read_bytes() if source.is_file() else fetch(item["url"], item["bytes"])
        if len(raw) != item["bytes"] or hashlib.sha256(raw).hexdigest() != item["sha256"]:
            raise ValueError("source mismatch: "+item["file"])
        if not source.exists():
            source.parent.mkdir(parents=True, exist_ok=True)
            source.write_bytes(raw)
    return lock


class Kit:
    def __init__(self, folder):
        self.name = folder.name
        self.doc = json.loads((folder/"model.gltf").read_bytes())
        self.blob = (folder/(folder.name+".bin")).read_bytes()

    def array(self, index):
        a = self.doc["accessors"][index]
        v = self.doc["bufferViews"][a["bufferView"]]
        dt = np.dtype({5126:"<f4",5123:"<u2",5125:"<u4"}[a["componentType"]])
        width = {"VEC2":2,"VEC3":3,"SCALAR":1}[a["type"]]
        return np.ndarray((a["count"],width), dt, buffer=self.blob,
                          offset=v.get("byteOffset",0)+a.get("byteOffset",0),
                          strides=(v.get("byteStride",width*dt.itemsize),dt.itemsize))

    def piece(self, nodes):
        # Kit nodes are laid out on a catalogue sheet. Individual modules use
        # the modelling pivot. Complete props retain all relative node transforms.
        parts = []
        matched = set()
        for node in self.doc["nodes"]:
            if nodes and node["name"] not in nodes:
                continue
            matched.add(node["name"])
            if "children" in node or "matrix" in node:
                raise ValueError("nested kit needs a scene graph transform: "+node["name"])
            linear = rotation(node.get("rotation",[0,0,0,1])) @ np.diag(node.get("scale",[1,1,1]))
            for p in self.doc["meshes"][node["mesh"]]["primitives"]:
                if p.get("mode",4) != 4:
                    raise ValueError("expected indexed triangles")
                mat = self.doc["materials"][p["material"]]
                tex = mat["pbrMetallicRoughness"].get("baseColorTexture")
                if tex:
                    img = self.doc["textures"][tex["index"]]["source"]
                    surface = Path(self.doc["images"][img]["uri"]).name.removesuffix("_diff_1k.jpg")
                else:
                    surface = "Window"  # unpowered laptop display
                xyz = self.array(p["attributes"]["POSITION"]) @ linear.T
                if not nodes:
                    xyz += node.get("translation",[0,0,0])
                normal = self.array(p["attributes"]["NORMAL"]) @ np.linalg.inv(linear)
                normal /= np.linalg.norm(normal,axis=1,keepdims=True).clip(1e-8)
                # glTF Y-up to forward/left/up; the native OBJ importer converts FLU to UE.
                xyz = xyz[:,[0,2,1]] * [1,-1,1]
                normal = normal[:,[0,2,1]] * [1,-1,1]
                uv = self.array(p["attributes"]["TEXCOORD_0"]).copy()
                uv[:,1] = 1-uv[:,1]
                tri = self.array(p["indices"]).reshape(-1,3)
                if np.linalg.det(linear)<0:
                    tri = tri[:,::-1]
                parts.append((surface,xyz,normal,uv,tri))
        if not parts or (nodes and matched != set(nodes)):
            raise ValueError("missing kit nodes: "+str(nodes))
        return parts


def save_piece(path, parts):
    slots = {}
    points = []
    offset = 1
    with path.open("w",encoding="ascii",newline="\n") as f:
        f.write("# Metric Poly Haven kit; centimetres, FLU, CC0-1.0\ng surface\n")
        for surface,xyz,normals,uv,tri in parts:
            slot = slots.setdefault(surface,"surface%02d"%len(slots))
            np.savetxt(f,xyz*100,fmt="v %.6f %.6f %.6f")
            np.savetxt(f,normals,fmt="vn %.7f %.7f %.7f")
            np.savetxt(f,uv,fmt="vt %.7f %.7f")
            f.write("usemtl "+slot+"\n")
            np.savetxt(f,np.repeat(tri+offset,3,axis=1),fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")
            points.append(xyz)
            offset += len(xyz)
    return {v:k for k,v in slots.items()},np.concatenate(points),sum(len(p[-1]) for p in parts)


def distant_tree(parts):
    """Cluster canopy samples at 5 cm for tree instances outside the sensor range."""
    lod = []
    for surface,xyz,normals,uv,tri in parts:
        if not surface.endswith("_twigs"):
            lod.append((surface,xyz,normals,uv,tri))
            continue
        cells = np.floor(xyz/.05).astype(np.int32)
        _,indices,remap = np.unique(cells,axis=0,return_index=True,return_inverse=True)
        faces = remap.reshape(-1)[tri]
        a,b,c = faces.T
        faces = faces[(a!=b)&(b!=c)&(a!=c)]
        _,unique_faces = np.unique(np.sort(faces,axis=1),axis=0,return_index=True)
        faces = faces[np.sort(unique_faces)]
        p = xyz[indices][faces]
        faces = faces[np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1)>1e-8]
        used,compact = np.unique(faces,return_inverse=True)
        vertices = indices[used]
        lod.append((surface,xyz[vertices],normals[vertices],uv[vertices],compact.reshape(-1,3)))
    return lod


def bake(root=ROOT, output=None):
    lock = restore(root)
    output = output or root
    derived = output/"derived"
    derived.mkdir(exist_ok=True)
    manifest, kits = {}, {}
    finishes = {s for a in lock["assets"].values() for s in a["surfaces"]}

    def record(name, points, triangles, **extra):
        path = derived/(name+".obj")
        ue_points = np.asarray(points)*[1,-1,1]
        manifest[name] = dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                              vertices=len(points),triangles=triangles,
                              bounds_ue_m=[ue_points.min(axis=0).tolist(),ue_points.max(axis=0).tolist()],
                              instances=[],**extra)

    for environment in ("indoor","outdoor"):
        layers = defaultdict(Mesh)
        def instance(kit, nodes, origin, yaw=0, scale=(1,1,1), pitch=0, roll=0):
            nodes = (nodes,) if isinstance(nodes,str) else tuple(nodes)
            suffix = nodes[0].removeprefix(kit+"_") if nodes else ""
            if len(nodes)>1:
                suffix += "_assembled"
            name = "kit_"+kit+("_"+suffix if suffix else "")
            if name not in manifest:
                if kit not in kits:
                    kits[kit] = Kit(root/"source"/kit)
                parts = kits[kit].piece(nodes)
                if kit=="fir_sapling_medium":
                    parts = distant_tree(parts)
                slots,xyz,tris = save_piece(derived/(name+".obj"),parts)
                if set(slots.values()) - finishes - {"Window"}:
                    raise ValueError("unreviewed material: "+str(slots))
                record(name,xyz,tris,slots=slots,source=kit,nodes=nodes)
            if list(manifest[name]["nodes"]) != list(nodes):
                raise ValueError("ambiguous module name")
            manifest[name]["instances"].append(dict(environment=environment,origin_m=origin,
                                                    yaw_deg=yaw,pitch_deg=pitch,roll_deg=roll,scale=scale))
        assemble(environment,instance,layers,root)
        for finish,mesh in layers.items():
            name = "shell_"+environment+"_"+finish
            mesh.save(derived/(name+".obj"),site_coordinates=True)
            record(name,mesh.vertices,len(mesh.faces),finish=finish,source="Robotic authored shell")
            manifest[name]["instances"].append(dict(environment=environment,origin_m=[0,0,0],
                                                    yaw_deg=0,pitch_deg=0,roll_deg=0,scale=[1,1,1]))
    (output/"meshes.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf8",newline="\n")
    print(len(manifest),"meshes,",sum(len(m["instances"]) for m in manifest.values()),"placements")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify",action="store_true",help="rebake into a temporary directory and compare every output byte")
    if parser.parse_args().verify:
        with TemporaryDirectory() as tmp:
            fresh = Path(tmp)
            bake(ROOT,fresh)
            expected = {p.name for p in (ROOT/"derived").glob("*.obj")}
            if expected != {p.name for p in (fresh/"derived").glob("*.obj")}:
                raise ValueError("derived mesh set mismatch")
            for name in [Path("meshes.json")]+[Path("derived")/n for n in sorted(expected)]:
                if (ROOT/name).read_bytes() != (fresh/name).read_bytes():
                    raise ValueError("rebake changed "+str(name))
            print("Verified",len(expected)+1,"outputs byte for byte")
    else:
        bake()
