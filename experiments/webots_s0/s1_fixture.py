"""Small, dependency-free guards for this native-node fixture, not a VRML parser."""
import ast
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
BASE = ROOT / "worlds/s0_search.wbt"
OCCLUSION = ROOT / "worlds/s1_occlusion.wbt"


def blocks(text, kind):
    # These fixtures contain no braces in strings. Comments are ignored.
    text = re.sub(r"#.*", "", text)
    result = []
    for match in re.finditer(r"\b" + re.escape(kind) + r"\s*\{", text):
        depth, end = 1, match.end()
        while depth and end < len(text):
            depth += (text[end] == "{") - (text[end] == "}")
            end += 1
        if depth:
            raise ValueError("Unclosed " + kind)
        result.append(text[match.start():end])
    return result


def field(node, name):
    # Only direct scalar fields: skip nested nodes and MF arrays.
    depth = brackets = 0
    lines = []
    for line in node.splitlines()[1:]:
        if depth == 0 and brackets == 0 and "{" not in line and "[" not in line:
            lines.append(line)
        depth += line.count("{") - line.count("}")
        brackets += line.count("[") - line.count("]")
    values = re.findall(r"^\s*" + re.escape(name) + r"\s+(.+?)\s*$", "\n".join(lines), re.M)
    if len(values) != 1:
        raise ValueError(f"Expected one direct field {name}, got {len(values)}")
    return values[0]


def expect(node, name, value):
    actual = field(node, name)
    if isinstance(value, str):
        if actual != value:
            raise ValueError(f"{name}: {actual!r} != {value!r}")
    else:
        expected = value if isinstance(value, tuple) else (value,)
        numbers = tuple(map(float, actual.split()))
        if len(numbers) != len(expected) or any(
                not math.isclose(a, b, rel_tol=0, abs_tol=1e-10) for a, b in zip(numbers, expected)):
            raise ValueError(f"{name}: {numbers} != {expected}")


def make_occlusion(base):
    """Same small room/UGV; stationary camera faces a person behind an opaque wall."""
    world = base.replace("rotation 0 0 1 -1.5707963267948966", "rotation 0 0 1 0", 1)
    world = world.replace('controller "s0_smoke"', 'controller "s1_occlusion"\n  supervisor TRUE', 1)
    return world + '''
# S1: visual AND collision geometry, opaque and taller/wider than the person.
DEF OCCLUDER Solid {
  translation -0.5 -1 1.1
  children [
    Shape {
      appearance Appearance {
        material Material { diffuseColor 0.25 0.35 0.45 }
      }
      geometry DEF OCCLUDER_BOX Box {
        size 0.1 2 2.2
      }
    }
  ]
  name "S1 opaque wall"
  boundingObject USE OCCLUDER_BOX
}
'''


def validate(text, occlusion=False):
    if not text.startswith("#VRML_SIM R2025a utf8"):
        raise ValueError("Expected Webots R2025a")
    if re.search(r"\b(linearVelocity|angularVelocity)\b", text):
        raise ValueError("Saved physics velocity is forbidden")
    if re.search(r"\b(EXTERNPROTO|Mesh|ImageTexture)\b", text):
        raise ValueError("S1 fixture must stay native and offline")
    info, = blocks(text, "WorldInfo")
    for key, value in dict(basicTimeStep=32, coordinateSystem='"ENU"', randomSeed=0).items():
        expect(info, key, value)
    robot, = blocks(text, "Robot")
    expect(robot, "translation", (-2, -1, .1))
    expect(robot, "rotation", (0, 0, 1, 0 if occlusion else -math.pi / 2))
    expect(robot, "controller", '"s1_occlusion"' if occlusion else '"s0_smoke"')
    if "DEF S0_ROBOT Robot" not in text:
        raise ValueError("Missing robot DEF")
    for kind in ("HingeJointParameters", "BallJointParameters"):
        joints = blocks(robot, kind)
        if len(joints) != 2:
            raise ValueError("Expected two " + kind)
        if any(re.search(r"\bposition\b", joint) for joint in joints):
            raise ValueError("Saved joint position is forbidden")
    for node, y in zip(blocks(robot, "HingeJointParameters"), (.18, -.18)):
        expect(node, "anchor", (0, y, 0))
        expect(node, "axis", (0, 1, 0))
    solids = {field(n, "name").strip('"'): n for n in blocks(text, "Solid")}
    for name, position, size in (
        ("floor", (0, 0, -.05), (8, 6, .1)),
        ("east wall", (4, 0, .3), (.1, 6, .6)),
        ("west wall", (-4, 0, .3), None),
        ("north wall", (0, 3, .3), (8, .1, .6)),
        ("south wall", (0, -3, .3), None),
        ("obstacle", (.3, .8, .6), (.6, .6, 1.2)),
        ("person", (1, -1, .9), (.3, .45, .7)),
    ):
        node = solids[name]
        expect(node, "translation", position)
        if "physics Physics" in node or "boundingObject" not in node:
            raise ValueError("Expected static collision geometry: " + name)
        if size:
            expect(blocks(node, "Box")[0], "size", size)
    expect(solids["west wall"], "boundingObject", "USE WALL_X")
    expect(solids["south wall"], "boundingObject", "USE WALL_Y")
    expect(solids["person"], "model", '"s0_person"')
    for name, y in (("left wheel solid", .18), ("right wheel solid", -.18)):
        expect(solids[name], "translation", (0, y, 0))
        expect(solids[name], "rotation", (1, 0, 0, math.pi / 2))
    for name, x in (("front caster", .16), ("rear caster", -.16)):
        expect(solids[name], "translation", (x, 0, -.07))
        if re.search(r"\brotation\b", solids[name]):
            raise ValueError("Caster must have default initial rotation")
    wheel, = blocks(robot, "Cylinder")
    expect(wheel, "radius", .1)
    camera, = blocks(robot, "Camera")
    for key, value in dict(translation=(.12, 0, .55), name='"camera"', width=160,
                           height=120, fieldOfView=1.012290966, near=.02, far=6).items():
        expect(camera, key, value)
    recognition, = blocks(camera, "Recognition")
    expect(recognition, "maxRange", 3.5)
    expect(recognition, "occlusion", 1)
    lidar, = blocks(robot, "Lidar")
    for key, value in dict(translation=(.12, 0, .15), name='"lidar"', type='"fixed"',
                           projection='"cylindrical"', noise=0, horizontalResolution=180,
                           numberOfLayers=1, fieldOfView=2 * math.pi, verticalFieldOfView=.01,
                           near=.02, minRange=.05, maxRange=6).items():
        expect(lidar, key, value)
    if occlusion:
        wall = solids["S1 opaque wall"]
        expect(wall, "translation", (-.5, -1, 1.1))
        expect(blocks(wall, "Box")[0], "size", (.1, 2, 2.2))
        expect(wall, "boundingObject", "USE OCCLUDER_BOX")
        if "transparency" in wall or "physics Physics" in wall:
            raise ValueError("Occluder must be opaque and static")


def validate_files():
    base = BASE.read_text(encoding="utf-8")
    validate(base)
    scenario = OCCLUSION.read_text(encoding="utf-8")
    validate(scenario, occlusion=True)
    if scenario != make_occlusion(base):
        raise ValueError("Occlusion world drifted; regenerate with run_s1.py --write-occlusion")
    # Read constants without importing the Webots API outside a controller process.
    tree = ast.parse((ROOT / "controllers/s0_smoke/s0_smoke.py").read_text(encoding="utf-8"))
    constants = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
                 if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)}
    for key, expected in dict(SENSOR_MS=96, RADIUS_M=.1, TRACK_M=.36,
                               STOP_RANGE_M=.8, TIMEOUT_S=90.).items():
        if constants.get(key) != expected:
            raise ValueError("Controller constant drift: " + key)
