"""Bake CC0 source meshes and original fractured structural meshes to UE OBJ.

Original authored geometry is dedicated to CC0-1.0. No engine assets or AI
images are embedded. UVs on original structures have a two metre repeat.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from experiments.ue5_s4.bake_humans import rotation
from experiments.ue5_s4.prepare_environment import BASE, MODELS


class Mesh:
    def __init__(self):
        self.vertices, self.uvs, self.faces = [], [], []

    def triangle(self, points):
        points = np.asarray(points, dtype=float)
        normal = np.cross(points[1]-points[0], points[2]-points[0])
        if np.linalg.norm(normal) < 1e-9:
            return
        axis = int(np.argmax(np.abs(normal)))
        uv_axes = [i for i in range(3) if i != axis]
        start = len(self.vertices)
        self.vertices.extend(points.tolist())
        self.uvs.extend((points[:, uv_axes] / 2).tolist())
        self.faces.append([start, start+1, start+2])

    def quad(self, a, b, c, d):
        self.triangle([a, b, c]); self.triangle([a, c, d])

    def wall(self, profile, thickness=.24):
        # Profile is a monotone top edge, left to right. Each interval is a
        # closed trapezoidal prism, with explicit fracture faces and thickness.
        for (x, z), (xx, zz) in zip(profile, profile[1:]):
            a,b,c,d = (x,-thickness/2,0),(xx,-thickness/2,0),(xx,-thickness/2,zz),(x,-thickness/2,z)
            e,f,g,h = [(p[0],thickness/2,p[2]) for p in (a,b,c,d)]
            for face in ((a,b,c,d),(f,e,h,g),(e,a,d,h),(b,f,g,c),(d,c,g,h),(e,f,b,a)):
                self.quad(*face)

    def tube(self, points, radius=.012, sides=10):
        points = np.asarray(points)
        rings = []
        for i,p in enumerate(points):
            tangent = points[min(i+1,len(points)-1)]-points[max(0,i-1)]
            tangent = tangent / np.linalg.norm(tangent)
            axis = np.array([0.,0.,1.]) if abs(tangent[2])<.9 else np.array([1.,0.,0.])
            a = np.cross(tangent,axis); a /= np.linalg.norm(a)
            b = np.cross(tangent,a)
            rings.append([p+radius*(a*math.cos(j*math.tau/sides)+b*math.sin(j*math.tau/sides)) for j in range(sides)])
        for ring,next_ring in zip(rings,rings[1:]):
            for j in range(sides):
                k=(j+1)%sides
                self.quad(ring[j],ring[k],next_ring[k],next_ring[j])

    def shard(self, center, size, phase=0):
        # Bevelled, asymmetric concrete fragment, not a scaled engine cube.
        rings=[]
        for z,scale in ((0,.65),(.12,1.),(.78,.92),(1.,.55)):
            ring=[]
            for j in range(7):
                a=j*math.tau/7+phase
                r=scale*(1+.12*math.sin(j*5+phase))
                ring.append([center[0]+size[0]*.5*r*math.cos(a),center[1]+size[1]*.5*r*math.sin(a),center[2]+size[2]*z])
            rings.append(ring)
        for r,s in zip(rings,rings[1:]):
            for j in range(7):self.quad(r[j],r[(j+1)%7],s[(j+1)%7],s[j])
        for ring,reverse in ((rings[0],True),(rings[-1],False)):
            for j in range(1,6):
                self.triangle([ring[0],ring[j+1],ring[j]] if reverse else [ring[0],ring[j],ring[j+1]])

    def save(self, path):
        # Canonical FLU metres -> UE OBJ centimetres. Reflection reverses faces.
        lines=["# Original Robotic S4 structural geometry, CC0-1.0", "g surface", "usemtl surface"]
        lines += ["v %.7f %.7f %.7f" % (x*100,-y*100,z*100) for x,y,z in self.vertices]
        lines += ["vt %.7f %.7f" % tuple(uv) for uv in self.uvs]
        lines += ["f " + " ".join(f"{i+1}/{i+1}" for i in reversed(face)) for face in self.faces]
        path.write_text("\n".join(lines)+"\n",encoding="utf-8",newline="\n")


def convert(name, destination):
    folder=BASE / "source" / name
    doc=json.loads((folder/"model.gltf").read_text())
    buffers=[(folder/Path(b["uri"]).name).read_bytes() for b in doc["buffers"]]
    def array(index):
        item=doc["accessors"][index]; view=doc["bufferViews"][item["bufferView"]]
        dtype=np.dtype({5126:"<f4",5123:"<u2",5125:"<u4"}[item["componentType"]])
        width={"VEC2":2,"VEC3":3,"SCALAR":1}[item["type"]]
        return np.ndarray((item["count"],width),dtype=dtype,buffer=buffers[view["buffer"]],
            offset=view.get("byteOffset",0)+item.get("byteOffset",0),strides=(view.get("byteStride",dtype.itemsize*width),dtype.itemsize)).copy()
    vertices,uvs,faces=[],[],[]
    def visit(index,parent):
        node=doc["nodes"][index]; matrix=np.eye(4)
        if "matrix" in node:matrix=np.array(node["matrix"]).reshape(4,4).T
        else:
            matrix[:3,:3]=rotation(node.get("rotation",[0,0,0,1]))@np.diag(node.get("scale",[1,1,1]))
            matrix[:3,3]=node.get("translation",[0,0,0])
        matrix=parent@matrix
        if "mesh" in node:
            for part in doc["meshes"][node["mesh"]]["primitives"]:
                if part.get("mode",4)!=4:raise ValueError("expected triangle mesh")
                p=array(part["attributes"]["POSITION"])
                p=(matrix@np.column_stack((p,np.ones(len(p)))).T).T[:,:3]
                offset=len(vertices)
                vertices.extend(p[:,[2,0,1]].tolist())
                uv=array(part["attributes"]["TEXCOORD_0"]);uv[:,1]=1-uv[:,1]
                uvs.extend(uv.tolist())
                faces.extend((array(part["indices"]).reshape(-1,3)+offset).tolist())
        for child in node.get("children",[]):visit(child,matrix)
    for node in doc["scenes"][doc.get("scene",0)]["nodes"]:visit(node,np.eye(4))
    points=np.asarray(vertices);points[:,2]-=points[:,2].min();points[:,:2]-=(points[:,:2].min(axis=0)+points[:,:2].max(axis=0))/2
    # Coordinates above are already UE-oriented (cyclic permutation, no reflection).
    lines=[f"# Poly Haven {name}, CC0-1.0; glTF node transforms baked", "g surface", "usemtl surface"]
    lines += ["v %.7f %.7f %.7f" % tuple(p*100) for p in points]
    lines += ["vt %.7f %.7f" % tuple(uv) for uv in uvs]
    lines += ["f " + " ".join(f"{i+1}/{i+1}" for i in face) for face in faces]
    destination.write_text("\n".join(lines)+"\n",encoding="utf-8",newline="\n")


def bake():
    lock=json.loads((BASE/"lock.json").read_text())
    for item in lock["files"]:
        if hashlib.sha256((BASE/"source"/item["file"]).read_bytes()).hexdigest()!=item["sha256"]:
            raise ValueError("source mismatch: "+item["file"])
    derived=BASE/"derived";derived.mkdir(exist_ok=True)
    for name in MODELS:convert(name,derived/(name+".obj"))
    wall=Mesh();wall.wall([(-1.5,2.6),(-1.15,2.6),(-.98,2.42),(-.8,1.65),(-.6,1.75),(-.38,.85),(-.1,.72),(.2,.9),(.45,1.3),(.7,1.22),(1.,2.2),(1.2,2.45),(1.5,2.6)])
    wall.save(derived/"fractured_wall.obj")
    rubble=Mesh()
    for i in range(28):
        a=i*2.399963; r=.12+.42*math.sqrt(i/28)
        rubble.shard([r*math.cos(a),r*.7*math.sin(a),.04*(1-i/28)], [.24+.11*math.sin(i)**2,.17+.13*math.cos(i)**2,.09+.14*(1-i/28)],i)
    rubble.save(derived/"rubble_heap.obj")
    slab=Mesh();slab.shard([0,0,0],[2.,1.3,.16],.3);slab.save(derived/"fallen_slab.obj")
    rebar=Mesh()
    for i in range(5):
        x=-.4+i*.2
        rebar.tube([[x,0,0],[x,.01,.5],[x+.04,.04,.9],[x+.16,.08,1.15],[x+.19,.1,1.25]])
    for z in (.2,.5,.8):rebar.tube([[-.45,0,z],[.45,0,z]])
    rebar.save(derived/"exposed_rebar.obj")
    manifest={}
    for path in sorted(derived.glob("*.obj")):
        verts=np.array([[float(x) for x in line.split()[1:]] for line in path.read_text().splitlines() if line.startswith("v ")])/100
        manifest[path.stem]={"file":path.name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "vertices":len(verts),"triangles":sum(line.startswith("f ") for line in path.read_text().splitlines()),
            "bounds_ue_m":[verts.min(axis=0).tolist(),verts.max(axis=0).tolist()],
            "license":"CC0-1.0","source":lock["assets"].get(path.stem,{"source":"Original Robotic S4 authored mesh"})["source"]}
    (BASE/"meshes.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    print("Baked environment meshes:",len(manifest))


if __name__ == "__main__":bake()
