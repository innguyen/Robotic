"""Small private dev scenarios, not the S3 48-family corpus or agent inputs."""


def scenario(kind="clean", seed=0, perception=0, odometry=0):
    if kind not in ("clean", "occluded", "absent", "multi", "sensor_black"):
        raise ValueError("unknown scenario")
    victims = [] if kind == "absent" else [{"gt_id": "v0", "anchor_m": [3.0, .4, .65]}]
    if kind == "multi":
        victims.append({"gt_id": "v1", "anchor_m": [3.4, -1.1, .65]})
    boxes = [
        {"id": "back_wall", "center_m": [5.5, 0, 1.0], "size_m": [.2, 7.0, 2.0]},
        {"id": "rubble", "center_m": [1.8, -1.8, .2], "size_m": [.7, .5, .4]},
    ]
    if kind == "occluded":
        boxes.append({"id": "occluder", "center_m": [2.5, .68, .65], "size_m": [.15, .42, 1.1]})
    return {"schema_version": "sar-s4-mvp-0.1", "kind": kind, "seed": seed,
            "perception": perception, "odometry": odometry, "family": "primitive_room",
            "pair_id": f"{kind}-{seed}", "start_m": [0.0, 0.0, .2], "start_yaw": -.75,
            "boxes": boxes, "victims": victims, "sensor_black": kind == "sensor_black"}


def sar_scenario(kind="clean", seed=0, perception=0, odometry=0, environment="indoor", posture="standing", depth_model="structured", scene_set="legacy"):
    import json
    from pathlib import Path
    if scene_set not in ("legacy","assets"):raise ValueError("unknown scene set")
    pose_path="anatomy/dressed/poses.json" if scene_set=="assets" else "derived/poses.json"
    poses = json.loads((Path(__file__).resolve().parents[1] / "experiments/ue5_s4/assets" / pose_path).read_text())
    config = scenario("clean" if kind in ("obstacles", "blocked") else kind, seed, perception, odometry)
    config.update(kind=kind, profile="sar_human_v1", environment=environment, depth_model=depth_model,
                  family="original_rescue_layout_v1",
                  pair_id=f"{environment}-{kind}-{posture}-{seed}", start_yaw=0.)
    config["boxes"] = [
        {"id": "wall_back", "center_m": [6.5, 0, 1.3], "size_m": [.2, 8., 2.6], "material": "Brick"},
        {"id": "wall_left", "center_m": [3, 4., 1.3], "size_m": [7., .2, 2.6]},
        {"id": "broken_wall_right", "center_m": [4.8, -4., .7], "size_m": [3., .2, 1.4]},
        {"id": "rubble_a", "center_m": [2., -1.6, .16], "size_m": [.9, .7, .32]},
        {"id": "rubble_b", "center_m": [3.1, 2., .22], "size_m": [.6, .8, .44]},
        {"id": "column", "center_m": [4., -2., 1.2], "size_m": [.3, .3, 2.4]},
        {"id": "rear_wall", "center_m": [-.8, 0, 1.3], "size_m": [.15, 8.4, 2.6]},
        {"id": "right_boundary", "center_m": [3, -4.2, .35], "size_m": [7.8, .15, .7]},
    ]
    if environment == "outdoor":
        config["boxes"][1]["size_m"][2] = .65
        config["boxes"][1]["center_m"][2] = .325
    if scene_set=="assets":
        from .asset_scenarios import asset_layout
        config["boxes"],target = asset_layout(environment)
        config.update(scene_variant=environment+"_damaged_assets_v2",family=environment+"_damaged_assets_v2")
    config["victims"] = [] if kind == "absent" else [{"gt_id": "v0", "anchor_m": [4., .5, poses[posture]["torso_height_m"]], "posture": posture, "yaw_deg": 90. if posture == "prone" else 180.}]
    if scene_set=="assets" and config["victims"]:config["victims"][0]["anchor_m"][:2]=target
    if kind == "multi":
        config["victims"].append({"gt_id": "v1", "anchor_m": [3., -2.8, poses["seated"]["torso_height_m"]], "posture": "seated", "yaw_deg": 140.})
    if kind == "occluded":
        config["boxes"].append({"id": "partition", "center_m": [2.8, .85, .8], "size_m": [.18, .9, 1.6]})
    if kind == "obstacles":
        # A feasible detour with no initial direct view, distinct from partial
        # occlusion and the deliberately infeasible full-width blocked case.
        config["boxes"].append({"id":"detour_partition","center_m":[2.4,0,1.3],"size_m":[.2,3.2,2.6]})
        if config["victims"]: config["victims"][0]["anchor_m"][:2] = [4.8,.8]
    if kind == "blocked":
        config["boxes"].append({"id": "impassable_partition", "center_m": [2.5, 0, 1.3], "size_m": [.2, 8., 2.6]})
    return config
