"""Bake licensed glTF skin into reproducible static UE-coordinate pose meshes.

No Blender, proprietary content or engine binary is needed. Original mesh:
Cesium Man, Cesium (2017), CC BY 4.0. Derived clothing materials replace logo.
"""
import hashlib
import json
from pathlib import Path
import numpy as np

BASE = Path(__file__).parent / "assets"


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def align(source, target):
    a, b = source / np.linalg.norm(source), target / np.linalg.norm(target)
    v, c = np.cross(a, b), np.dot(a, b)
    if c < -.99999:
        basis = np.array([1., 0., 0.]) if abs(a[0]) < .9 else np.array([0., 0., 1.])
        axis = np.cross(a, basis); axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    cross = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + cross + cross @ cross / max(1e-9, 1 + c)


def bake():
    document = json.loads((BASE / "source/CesiumMan.gltf").read_text())
    binary = (BASE / "source/CesiumMan_data.bin").read_bytes()
    def array(index):
        a = document["accessors"][index]; view = document["bufferViews"][a["bufferView"]]
        dtype = {5126: "<f4", 5123: "<u2", 5121: "u1", 5125: "<u4"}[a["componentType"]]
        size = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}[a["type"]]
        offset = view.get("byteOffset", 0) + a.get("byteOffset", 0)
        stride = view.get("byteStride", np.dtype(dtype).itemsize * size)
        return np.ndarray((a["count"], size), dtype=dtype, buffer=binary, offset=offset,
                          strides=(stride, np.dtype(dtype).itemsize)).copy()
    nodes = document["nodes"]
    local = []
    parents = {}
    for index, node in enumerate(nodes):
        m = np.eye(4)
        if "matrix" in node:
            m = np.array(node["matrix"]).reshape(4, 4).T
        else:
            m[:3, :3] = rotation(node.get("rotation", [0, 0, 0, 1])) @ np.diag(node.get("scale", [1, 1, 1]))
            m[:3, 3] = node.get("translation", [0, 0, 0])
        local.append(m)
        for child in node.get("children", []): parents[child] = index
    def globals_of(transforms):
        result = {}
        def get(i):
            if i not in result: result[i] = (get(parents[i]) if i in parents else np.eye(4)) @ transforms[i]
            return result[i]
        for i in range(len(nodes)): get(i)
        return result
    original = globals_of(local)
    skin = document["skins"][0]
    inverse_bind = array(skin["inverseBindMatrices"]).reshape(-1, 4, 4).transpose(0, 2, 1)
    primitive = document["meshes"][0]["primitives"][0]
    attrs = primitive["attributes"]
    positions = array(attrs["POSITION"]); indices = array(primitive["indices"]).flatten().reshape(-1, 3)
    joints, weights = array(attrs["JOINTS_0"]).astype(int), array(attrs["WEIGHTS_0"])
    homogeneous = np.column_stack((positions, np.ones(len(positions))))
    names = {node.get("name"): i for i, node in enumerate(nodes)}
    export = BASE / "derived"; export.mkdir(exist_ok=True)
    meta = {}
    for pose in ("standing", "seated", "prone"):
        transforms = [m.copy() for m in local]
        for shoulder, elbow, hand in (("Skeleton_arm_joint_R", "Skeleton_arm_joint_R__2_", "Skeleton_arm_joint_R__3_"),
                                      ("Skeleton_arm_joint_L__4_", "Skeleton_arm_joint_L__3_", "Skeleton_arm_joint_L__2_")):
            for first, second in ((shoulder, elbow), (elbow, hand)):
                i, child = names[first], names[second]
                current = globals_of(transforms)
                source = current[child][:3, 3] - current[i][:3, 3]
                world_r = align(source, np.array([0., -1., 0.])) @ current[i][:3, :3]
                transforms[i][:3, :3] = np.linalg.inv(current[parents[i]][:3, :3]) @ world_r
        # Set anatomical limb directions in glTF world (+Y up), retaining the
        # skinned topology and torso/head. Sitting has forward thighs/down shins.
        for side in ("R", "L"):
            for number, target in ((1, np.array([0., -1., 0.]) if pose != "seated" else np.array([0., 0., 1.])),
                                   (2, np.array([0., -1., 0.]))):
                i, child = names[f"leg_joint_{side}_{number}"], names[f"leg_joint_{side}_{number+1}"]
                current = globals_of(transforms)
                source = current[child][:3, 3] - current[i][:3, 3]
                world_r = align(source, target) @ current[i][:3, :3]
                transforms[i][:3, :3] = np.linalg.inv(current[parents[i]][:3, :3]) @ world_r
        current = globals_of(transforms)
        matrices = np.array([current[j] @ inverse_bind[k] for k, j in enumerate(skin["joints"])])
        posed = sum((np.einsum("nij,nj->ni", matrices[joints[:, channel]], homogeneous) * weights[:, channel, None])
                    for channel in range(4))[:, :3]
        # glTF right-handed Y-up -> UE left-handed cm (forward=-Z, right=X).
        vertices = np.column_stack((-posed[:, 2], posed[:, 0], posed[:, 1]))
        height_scale = 1.75 / (vertices[:, 2].max() - vertices[:, 2].min()) if pose == "standing" else scale
        if pose == "standing": scale = height_scale
        vertices *= scale
        anchor_world = current[names["torso_joint_3"]][:3, 3]
        anchor = np.array([-anchor_world[2], anchor_world[0], anchor_world[1]]) * scale
        if pose == "prone":
            r = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            vertices = vertices @ r.T; anchor = anchor @ r.T
        floor = vertices[:, 2].min()
        vertices[:, 2] -= floor; anchor[2] -= floor
        vertices[:, :2] -= anchor[:2]; anchor[:2] = 0
        # Origin is the canonical torso anchor; ground height is retained in meta.
        vertices -= anchor
        path = export / (pose + ".obj")
        groups = {"skin": [], "shirt": [], "trousers": []}
        joint_names = [nodes[i].get("name", "") for i in skin["joints"]]
        for triangle in indices:
            dominant = [joint_names[joints[v, int(np.argmax(weights[v]))]] for v in triangle]
            material = "skin" if any("neck" in x or "arm_joint" in x and ("3_" in x or "2_" in x) for x in dominant) else ("trousers" if any("leg_joint" in x for x in dominant) else "shirt")
            groups[material].append(triangle)
        lines = ["# Derived from Cesium Man (Cesium), CC-BY-4.0; original logo texture replaced"]
        lines += ["v %.7f %.7f %.7f" % tuple(v * 100) for v in vertices]
        for material, triangles in groups.items():
            lines += ["g " + material, "usemtl " + material]
            lines += ["f %d %d %d" % tuple(int(i) + 1 for i in t[::-1]) for t in triangles]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        meta[pose] = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                      "vertices": len(vertices), "triangles": len(indices), "torso_height_m": float(anchor[2]),
                      "bounds_relative_m": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]}
    (export / "poses.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(json.dumps(meta, indent=2))


if __name__ == "__main__": bake()
