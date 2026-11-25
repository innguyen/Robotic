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
