"""Runtime MVP scenario contract, separate from immutable S3 design fixtures."""
import math

from . import PROTOCOL
from .contracts import fields, finite, identifier, integer, require, vector


def validate_scenario(config):
    fields(config, ("schema_version", "kind", "seed", "perception", "odometry", "family", "pair_id",
                    "start_m", "start_yaw", "boxes", "victims", "sensor_black"), ("profile", "environment", "depth_model", "scene_variant"))
    human = config.get("profile") == "sar_human_v1"
    if "profile" in config:
        require(human and config.get("environment") in ("indoor", "outdoor"), "invalid SAR profile")
        require(config.get("depth_model", "structured") in ("ideal","gaussian","structured"), "invalid depth model")
    else:
        require("environment" not in config and "depth_model" not in config and "scene_variant" not in config, "human settings require SAR profile")
    assets="scene_variant" in config
    if assets:
        require(config["scene_variant"]==config["environment"]+"_damaged_assets_v1", "invalid asset scene")
    require(config["schema_version"] == PROTOCOL, "unsupported scenario version")
    require(config["kind"] in ("clean", "occluded", "absent", "multi", "sensor_black", "obstacles", "blocked"), "unsupported scenario kind")
    require(human or config["kind"] not in ("obstacles","blocked"), "navigation challenges require human profile")
    require(integer(config["seed"]) and config["seed"] <= 2147483647, "invalid seed")
    require(type(config["perception"]) is int and config["perception"] in (0, 1) and
            type(config["odometry"]) is int and config["odometry"] in (0, 1), "invalid factorial condition")
    require(identifier(config["family"]) and identifier(config["pair_id"]), "invalid pairing identity")
    require(vector(config["start_m"], 3) and config["start_m"][2] == .2 and finite(config["start_yaw"]) and
            -math.pi <= config["start_yaw"] <= math.pi, "invalid start pose")
    require(type(config["sensor_black"]) is bool and config["sensor_black"] == (config["kind"] == "sensor_black"), "invalid black control")
    require(type(config["boxes"]) is list and len(config["boxes"]) <= 30, "invalid boxes")
    seen = set()
    for box in config["boxes"]:
        fields(box, ("id", "center_m", "size_m"), (("material","mesh") if assets else ("material",)) if human else ())
        if "material" in box:
            require(box["material"] in ("Concrete", "Brick", "Metal") + (("Debris","concrete_road_barrier_02","painted_wooden_chair_02","wooden_crate_01") if assets else ()), "invalid material")
        if "mesh" in box:
            from .asset_scenarios import MESH_IDS
            require(box["mesh"] in MESH_IDS,"unreviewed scene mesh")
        require(identifier(box["id"]) and box["id"] not in seen, "invalid box identity")
        seen.add(box["id"])
        require(vector(box["center_m"], 3) and vector(box["size_m"], 3) and all(0 < v <= 20 for v in box["size_m"]), "invalid box geometry")
    require(type(config["victims"]) is list and len(config["victims"]) <= 3, "invalid victim count")
    seen = set()
    for victim in config["victims"]:
        fields(victim, ("gt_id", "anchor_m"), ("posture", "yaw_deg") if human else ())
        if human:
            require(victim.get("posture") in ("standing", "seated", "prone") and finite(victim.get("yaw_deg")), "invalid human pose")
        require(identifier(victim["gt_id"]) and victim["gt_id"] not in seen, "invalid victim identity")
        seen.add(victim["gt_id"])
        require(vector(victim["anchor_m"], 3), "invalid anchor")
    require(bool(config["victims"]) == (config["kind"] != "absent"), "absent/count mismatch")
    return config
