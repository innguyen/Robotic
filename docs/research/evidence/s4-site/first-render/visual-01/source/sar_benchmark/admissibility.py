"""Trusted pre-policy observability probes, separate from agent observations.

Conservative circular-footprint grid reachability is a static authoring check,
not a dynamics certificate. Visibility is checked twice on native UE captures.
"""
from collections import deque
import copy
import math


def reachable_grid(config, resolution=.3):
    radius = math.sqrt(2)*.2+.05
    def free(x,y):
        if not (-.45 <= x <= 6.15 and -3.75 <= y <= 3.65): return False
        for box in config["boxes"]:
            if box["center_m"][2]-box["size_m"][2]/2 >= .35: continue
            dx = max(0.,abs(x-box["center_m"][0])-box["size_m"][0]/2)
            dy = max(0.,abs(y-box["center_m"][1])-box["size_m"][1]/2)
            if math.hypot(dx,dy) < radius: return False
        return True
    start = tuple(round(value/resolution) for value in config["start_m"][:2])
    if not free(start[0]*resolution,start[1]*resolution): return {}
    distance = {start:0}; queue = deque([start])
    while queue:
        x,y = queue.popleft()
        for point in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if point not in distance and free(point[0]*resolution,point[1]*resolution):
                distance[point] = distance[x,y]+1; queue.append(point)
    return {(x*resolution,y*resolution):steps*resolution for (x,y),steps in distance.items()}


def probe(backend, config, max_views=16):
    reachable = reachable_grid(config)
    records, accepted = [], []
    for victim in config["victims"]:
        anchor = victim["anchor_m"]
        # One candidate per angular sector, then fill by shortest grid travel.
        # Selection uses geometry only and never detector outputs.
        candidates = sorted(((travel,x,y) for (x,y),travel in reachable.items()
                             if 1.7 <= math.dist((x,y),anchor[:2]) <= 6.5))
        diverse, sectors = [], set()
        for item in candidates:
            _,x,y = item
            sector = int((math.atan2(y-anchor[1],x-anchor[0])+math.pi)/math.tau*12)%12
            if sector not in sectors: diverse.append(item); sectors.add(sector)
        views = (diverse + [item for item in candidates if item not in diverse])[:max_views]
        success = False
        for travel,x,y in views:
            trial = copy.deepcopy(config); trial["start_m"] = [x,y,.2]
            trial["start_yaw"] = math.atan2(anchor[1]-y,anchor[0]-x)
            obs, first = backend.reset(trial,4)
            second_obs, second = backend.advance([0.,0.,0.])
            pixels = [truth["frame"]["visible"].get(victim["gt_id"],{}).get("pixels",0) for truth in (first,second)]
            success = min(pixels) >= 25
            records.append({"gt_id":victim["gt_id"],"start_m":trial["start_m"],"yaw":trial["start_yaw"],
                            "grid_path_m":travel,"epoch":obs["epoch"],"ticks":[obs["tick"],second_obs["tick"]],"visible_pixels":pixels,"accepted":success})
            if success: break
        if success: accepted.append(victim["gt_id"])
    backend.stop()
    return {"admissible":len(accepted)==len(config["victims"]) and bool(reachable),
            "accepted_victims":accepted,"reachable_grid_cells":len(reachable),"probes":records,
            "scope":"Native two-capture visibility plus conservative 2D circular-footprint connectivity; not wheel-dynamics reachability",
            "auxiliary_expected_infeasible":config["kind"]=="blocked"}
