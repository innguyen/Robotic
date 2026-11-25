"""Offline, small native-node Webots scenes. Generated worlds never edit S0/S1."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCENARIOS = json.loads((ROOT / "scenarios.json").read_text(encoding="utf-8"))


def solid(name, center, size, color="0.5 0.5 0.5", extra=""):
    xyz, dimensions = " ".join(map(str, center)), " ".join(map(str, size))
    extra_line = "  " + extra + "\n" if extra else ""
    return f'''DEF {name} Solid {{
  translation {xyz}
  name "{name}"
  children [ Shape {{
    appearance Appearance {{ material Material {{ diffuseColor {color} }} }}
    geometry DEF {name}_BOX Box {{ size {dimensions} }}
  }} ]
  boundingObject USE {name}_BOX
{extra_line}}}
'''


def world(name, policy_args=(), audit_args=()):
    cfg = SCENARIOS[name]
    x, y, yaw = cfg["start"]
    text = '''#VRML_SIM R2025a utf8
# S2 ideal holonomic velocity body. Collision dynamics, not roller/traction dynamics.
WorldInfo {
  basicTimeStep 32
  coordinateSystem "ENU"
  randomSeed 0
  # Direct velocity actuation must remain awake through mission dwell/pauses.
  physicsDisableTime 1000000
  FPS 30
  contactProperties [ ContactProperties {
    material1 "omni"
    material2 "default"
    coulombFriction [0]
    bounce 0
    bumpSound ""
    rollSound ""
    slideSound ""
  } ]
}
Viewpoint {
  orientation 0.2 0.9 0.4 1.5
  position 5 -6 7
  ambientOcclusionRadius 0
  bloomThreshold -1
}
Background { skyColor [0.18 0.22 0.28] }
DirectionalLight { direction -1 -1 -2 ambientIntensity 0.3 }
'''
    text += solid("FLOOR", (0, 0, -.05), (6, 6, .1))
    for label, center, size in (("EAST", (3, 0, .4), (.1, 6, .8)),
                                ("WEST", (-3, 0, .4), (.1, 6, .8)),
                                ("NORTH", (0, 3, .4), (6, .1, .8)),
                                ("SOUTH", (0, -3, .4), (6, .1, .8))):
        text += solid(label, center, size)
    for index, (bx, by, sx, sy, height) in enumerate(cfg["boxes"]):
        text += solid(f"BLOCK_{index}", (bx, by, height/2), (sx, sy, height), "0.5 0.3 0.1")
    if name == "calibration":
        text += solid("PERSON", (1.5, .2, .65), (.1, .1, .1), "0.8 0.1 0.2",
                      'model "s0_person" recognitionColors [0.8 0.1 0.2]')
        text += solid("MONITOR_BLOCK", (10, 10, .4), (.1, .6, .8))
    elif cfg["person"] is not None:
        # Same primitive mannequin as S0, cloned as fixture source, not a policy.
        source = (ROOT.parent / "webots_s0/worlds/s0_search.wbt").read_text(encoding="utf-8")
        person = source[source.index("DEF PERSON Solid {"):source.index("DEF S0_ROBOT Robot {")]
        px, py = cfg["person"]
        person = person.replace("translation 1 -1 0.9", f"translation {px} {py} 0.9", 1)
        text += person
    controller = "s2_calibrate" if name == "calibration" else "s2_core"
    args = " ".join(json.dumps(str(a)) for a in policy_args)
    text += f'''DEF S2_ROBOT Robot {{
  translation {x} {y} 0.1
  rotation 0 0 1 {yaw}
  name "S2 ideal omni"
  contactMaterial "omni"
  supervisor TRUE
  controller "{controller}"
  controllerArgs [{args}]
  children [
    Shape {{
      appearance Appearance {{ material Material {{ diffuseColor 0.1 0.3 0.8 }} }}
      geometry DEF BODY Box {{ size 0.4 0.4 0.2 }}
    }}
    Camera {{
      name "camera"
      translation 0.12 0 0.55
      width 160
      height 120
      fieldOfView 1.012290966
      near 0.02
      far 6
      recognition Recognition {{ maxRange 3.5 occlusion 1 }}
    }}
    Lidar {{
      name "lidar"
      translation 0.12 0 0.15
      type "fixed"
      projection "cylindrical"
      horizontalResolution 192
      numberOfLayers 1
      fieldOfView 6.283185307179586
      verticalFieldOfView 0.01
      near 0.02
      minRange 0.05
      maxRange 6
      noise 0
    }}
  ]
  boundingObject USE BODY
  physics Physics {{ density -1 mass 2 damping Damping {{ linear 0 angular 0 }} }}
}}
'''
    if audit_args:
        text += '\nRobot {\n  supervisor TRUE\n  controller "s2_audit"\n  controllerArgs ['
        text += " ".join(json.dumps(str(a)) for a in audit_args) + ']\n}\n'
    return text
