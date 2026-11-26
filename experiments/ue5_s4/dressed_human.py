"""Fit licensed HM08 garments to the neutral base, then apply its posed rig."""
import hashlib
from pathlib import Path
import numpy as np


def topology(path, group=None):
    uv, triangles = [], []
    current = None
    for line in path.read_text(encoding="utf-8").splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == "vt":
            uv.append(tuple(map(float, words[1:3])))
        elif words[0] == "g":
            current = words[1]
        elif words[0] == "f" and (group is None or current == group):
            face = [tuple(int(x)-1 for x in item.split("/")[:2]) for item in words[1:]]
            for i in range(1,len(face)-1):
                triangles.append((face[0],face[i],face[i+1]))
    return uv, triangles


def garment(source, name, points, matrices):
    lines = (source/(name+".mhclo")).read_text().splitlines()
    scale = np.ones(3)
    start = lines.index("verts 0")+1
    for line in lines[:start]:
        fields = line.split()
        if fields and fields[0] in ("x_scale","y_scale","z_scale"):
            axis = "xyz".index(fields[0][0])
            a,b = map(int,fields[1:3])
            scale[axis] = abs(points[a,axis]-points[b,axis])/float(fields[3])
    n = sum(l.startswith("v ") for l in (source/(name+".obj")).read_text().splitlines())
    rows = [line.split() for line in lines[start:] if line.strip()][:n]
    if len(rows) != n:
        raise ValueError("incomplete vertex binding for "+name)
    fitted = []
    for row in rows:
        if len(row) == 1:
            i = int(row[0])
            position, matrix = points[i], matrices[i]
        else:
            if len(row) != 9:
                raise ValueError("unsupported HM08 binding in "+name)
            refs = np.array(row[:3],int)
            weights = np.array(row[3:6],float)
            weights /= weights.sum()
            position = weights @ points[refs] + np.array(row[6:],float)*scale
            matrix = np.einsum("i,ijk->jk",weights,matrices[refs])
        if abs(matrix[3,3]-1) > .001:
            raise ValueError("binding refers to an unweighted helper in "+name)
        fitted.append((matrix @ np.r_[position,1])[:3])
    uv, faces = topology(source/(name+".obj"))
    return np.array(fitted), uv, faces


def bake_pose(base, posture, points, weights, transform, body_category, scale, torso):
    source = base/"source"
    matrices = np.zeros((len(points),4,4))
    for bone, items in weights.items():
        indices = np.array([i for i,_ in items])
        values = np.array([v for _,v in items])
        matrices[indices] += values[:,None,None]*transform(bone)
    totals = matrices[:,3,3].copy()
    weighted = totals > 0
    matrices[weighted] /= totals[weighted,None,None]
    # Some proxy helpers have no rig weights. Transfer the nearest rigged body
    # weights in neutral space, before any pose is applied.
    body = np.array(list(body_category))
    for index in np.flatnonzero(~weighted):
        nearest = body[np.argmin(np.sum((points[body]-points[index])**2,axis=1))]
        matrices[index] = matrices[nearest]
    posed = np.einsum("nij,nj->ni",matrices,np.column_stack([points,np.ones(len(points))]))[:,:3]
    uv, faces = topology(source/"base.obj","body")
    # Only exported skin patches can contribute to the silhouette. Clothes
    # and footwear have their own topology and UVs, posing in the same actor.
    faces = [face for face in faces if any(body_category[v] == "skin" for v,_ in face)]
    parts = [("skin",posed,uv,faces)]
    for name in ("shirt","trousers","footwear","eyes","bob"):
        parts.append((name,*garment(source,name,points,matrices)))
    lines = ["# HM08 licensed wardrobe, posed torso origin, centimetres","s 1"]
    material_faces = []
    vertices = []
    vertex_offset = uv_offset = 0
    for name, positions, texcoords, triangles in parts:
        used = sorted({v for face in triangles for v,_ in face})
        remap = {old:vertex_offset+i+1 for i,old in enumerate(used)}
        # MakeHuman Y-up -> UE X-forward/Y-right/Z-up. OBJ import preserves
        # the same winding and UV convention as the existing baked body.
        xyz = (positions[used]-torso)[:,[2,0,1]]*scale
        if posture == "prone":
            xyz = xyz[:,[2,1,0]]
            xyz[:,2] *= -1
        vertices.extend(xyz.tolist())
        lines.extend("vt %.7f %.7f"%tuple(pair) for pair in texcoords)
        material_faces.extend(("g "+name, "usemtl "+name))
        for face in triangles:
            material_faces.append("f "+" ".join(f"{remap[v]}/{uv_offset+t+1}" for v,t in face))
        vertex_offset += len(used)
        uv_offset += len(texcoords)
    xyz = np.array(vertices)
    lines.extend("v %.7f %.7f %.7f"%tuple(v*100) for v in xyz)
    lines.extend(material_faces)
    derived = base/"dressed"
    derived.mkdir(exist_ok=True)
    path = derived/(posture+".obj")
    path.write_text("\n".join(lines)+"\n",encoding="utf-8",newline="\n")
    lower, upper = xyz.min(axis=0), xyz.max(axis=0)
    return {"file":path.name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "vertices":len(vertices),"triangles":sum(len(f) for _,_,_,f in parts),
            "torso_height_m":float(-lower[2]),
            "bounds_relative_m":[[float(lower[0]),float(-upper[1]),float(lower[2])],
                                 [float(upper[0]),float(-lower[1]),float(upper[2])]],
            "material_slots":[name for name,*_ in parts],
            "human_source":"MakeHuman HM08 body and low-poly eyes; fitted CC0 wardrobe, hair and skin"}
