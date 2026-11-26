"""Bake MakeHuman CC0 anatomy/weights, with original loose work clothes.

Only body faces are exported. Helper/genital/rig geometry is excluded. This is
an anatomical development avatar, not a scan or photorealistic identity corpus.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from experiments.ue5_s4.bake_humans import align

BASE=Path(__file__).parent/"assets/anatomy"


def bake():
    lock=json.loads((BASE/"lock.json").read_text())
    for name,item in lock["files"].items():
        if hashlib.sha256((BASE/"source"/name).read_bytes()).hexdigest()!=item["sha256"]:
            raise ValueError("MakeHuman source hash mismatch")
    vertices,faces=[],[];group=None
    for line in (BASE/"source/base.obj").read_text().splitlines():
        parts=line.split()
        if not parts:continue
        if parts[0]=="v":vertices.append([float(v) for v in parts[1:4]])
        elif parts[0]=="g":group=parts[1]
        elif parts[0]=="f" and group=="body":
            polygon=[int(v.split('/')[0])-1 for v in parts[1:]]
            for j in range(1,len(polygon)-1):faces.append([polygon[0],polygon[j],polygon[j+1]])
    points=np.array(vertices);faces=np.array(faces)
    rig=json.loads((BASE/"source/default.mhskel").read_text())
    weights=json.loads((BASE/"source/default_weights.mhw").read_text())["weights"]
    joints={name:points[indices].mean(axis=0) for name,indices in rig["joints"].items()}
    bones=rig["bones"]
    active=np.unique(faces);rest=points.copy()
    # Garments follow the supplied skinning weights. Their surface has real
    # volume, cuffs and collar boundaries from topology, with a loose torso.
    dominant=[None]*len(points);maximum=np.zeros(len(points))
    for bone,items in weights.items():
        for index,value in items:
            if value>maximum[index]:maximum[index]=value;dominant[index]=bone
    def clothing(bone):
        if bone is None:raise ValueError("unweighted body vertex")
        if bone.startswith(("upperleg","lowerleg","pelvis")) or bone=="root":return "trousers"
        if bone.startswith(("spine","breast","clavicle","shoulder","upperarm")):return "shirt"
        if bone.startswith(("foot","toe")):return "boots"
        return "skin"
    categories=[clothing(dominant[i]) for i in active]
    category={i:c for i,c in zip(active,categories)}
    for i in active:
        c=category[i]
        if c in ("shirt","trousers"):
            # Expand outwards from the local bone axis; no image/GT changes.
            bone=bones[dominant[i]];a=joints[bone["head"]];b=joints[bone["tail"]]
            direction=b-a;projection=a+np.clip(np.dot(points[i]-a,direction)/np.dot(direction,direction),0,1)*direction
            radial=points[i]-projection;length=np.linalg.norm(radial)
            if length>1e-6:rest[i]+=radial/length*(.16 if c=="shirt" else .10)
    scale=1.75/np.ptp(points[active,1])
    derived=BASE/"derived";derived.mkdir(exist_ok=True)
    metadata={}
    for posture in ("standing","seated","prone"):
        transforms={}
        desired={}
        for side,sign in (("L",1),("R",-1)):
            desired["upperarm01."+side]=np.array([sign*.16,-1.,.03])
            desired["lowerarm01."+side]=np.array([0.,-1.,.12])
            if posture=="seated":
                desired["upperleg01."+side]=np.array([sign*.04,0.,1.])
                desired["lowerleg01."+side]=np.array([0.,-1.,0.])
                desired["upperarm01."+side]=np.array([sign*.15,-.5,.7])
                desired["lowerarm01."+side]=np.array([0.,-.3,.8])
        def transform(name):
            if name in transforms:return transforms[name]
            bone=bones[name];parent=transform(bone["parent"]) if bone["parent"] else np.eye(4)
            matrix=parent.copy()
            if name in desired:
                head,tail=joints[bone["head"]],joints[bone["tail"]]
                moved=(parent@np.r_[head,1])[:3]
                rotation=align(parent[:3,:3]@(tail-head),desired[name])@parent[:3,:3]
                matrix[:3,:3]=rotation;matrix[:3,3]=moved-rotation@head
            transforms[name]=matrix
            return matrix
        baked=np.zeros_like(points);totals=np.zeros(len(points))
        for bone,items in weights.items():
            indices=np.array([i for i,v in items]);values=np.array([v for i,v in items])
            moved=(transform(bone)@np.column_stack((rest[indices],np.ones(len(indices)))).T).T[:,:3]
            baked[indices]+=moved*values[:,None];totals[indices]+=values
        if np.any(totals[active]<.95):raise ValueError("incomplete anatomical skinning")
        baked[active]/=totals[active,None]
        anchor=(transform("spine02")@np.r_[joints[bones["spine02"]["head"]],1])[:3]
        # MH Y-up/front +Z -> canonical front +X/left +Y/up +Z.
        baked=baked[:,[2,0,1]]*scale;baked[:,1]*=-1
        anchor=anchor[[2,0,1]]*scale;anchor[1]*=-1
        if posture=="prone":
            # Prone, face down; arms remain along the body.
            r=np.array([[0,0,1],[0,1,0],[-1,0,0]])
            baked=baked@r.T;anchor=anchor@r.T
        floor=baked[active,2].min();baked[:,2]-=floor;anchor[2]-=floor
        baked-=anchor
        lines=["# MakeHuman base/rig/weights CC0-1.0; original loose garments; torso origin", "s 1"]
        remap={old:new for new,old in enumerate(active)}
        lines += ["v %.7f %.7f %.7f" % (baked[i,0]*100,-baked[i,1]*100,baked[i,2]*100) for i in active]
        for material in ("skin","shirt","trousers","boots"):
            lines.extend(["g "+material,"usemtl "+material])
            for face in faces:
                # Majority category avoids isolated clothing pixels at joints.
                labels=[category[int(i)] for i in face]
                selected=max(dict.fromkeys(labels),key=labels.count)
                if selected==material:
                    # MH -> canonical reflection plus canonical -> UE reflection
                    # cancel: preserve source winding.
                    lines.append("f "+" ".join(str(remap[int(i)]+1) for i in face))
        path=derived/(posture+".obj");path.write_text("\n".join(lines)+"\n",encoding="utf-8",newline="\n")
        metadata[posture]={"file":path.name,"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "vertices":len(active),"triangles":len(faces),"torso_height_m":float(anchor[2]),
            "bounds_relative_m":[baked[active].min(axis=0).tolist(),baked[active].max(axis=0).tolist()],
            "human_source":"MakeHuman CC0 anatomical base; original garment shaping",
            "limitations":"one neutral identity; no facial/skin texture scans, cloth simulation or appearance coverage"}
    (derived/"poses.json").write_text(json.dumps(metadata,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(metadata,indent=2))


if __name__=="__main__":bake()
