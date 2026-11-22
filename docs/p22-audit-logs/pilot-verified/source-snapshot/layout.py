"""rect-union-um-v1; byte-compatible with the P0 JavaScript reference."""
import hashlib
import json
import math

LIMIT = 2**53 - 1


def um(value):
    if type(value) not in (int, float) or abs(value) > LIMIT or not math.isfinite(value):
        raise ValueError('coordinate must be finite numeric')
    product = value * 1e6
    lower = math.floor(product)
    # Match Math.round ties toward +infinity without adding 0.5 to a large
    # already-integral double (which can round it to the next integer).
    scaled = lower + (product-lower >= 0.5)
    if abs(scaled) > LIMIT or scaled / 1e6 != value:
        raise ValueError('coordinate must have exact micrometre precision')
    return scaled


def rect(values):
    if not isinstance(values, list) or len(values) != 4:
        raise ValueError('rectangle requires four coordinates')
    out = list(map(um, values))
    if out[0] >= out[2] or out[1] >= out[3]:
        raise ValueError('inverted rectangle')
    return out


def canonical_layout(episode):
    x, y, xx, yy = rect(episode['bounds_m'])
    boxes = []
    for item in episode['obstacles']:
        a, b, c, d = rect(item['bounds_m'])
        boxes.append([a-x, b-y, c-x, d-y])
    if any(abs(v) > LIMIT for box in boxes for v in box) or max(xx-x, yy-y) > LIMIT:
        raise ValueError('coordinate span too large')
    xs = sorted({v for box in boxes for v in (box[0], box[2])})
    slabs = []
    for lo, hi in zip(xs, xs[1:]):
        merged = []
        for a, b in sorted((box[1], box[3]) for box in boxes if box[0] <= lo and box[2] >= hi):
            if merged and a <= merged[-1][1]:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        if not merged:
            continue
        if slabs and slabs[-1][1] == lo and slabs[-1][2] == merged:
            slabs[-1][1] = hi
        else:
            slabs.append([lo, hi, merged])
    return json.dumps(dict(bounds_um=[xx-x, yy-y], slabs=slabs, version='rect-union-um-v1'), separators=(',', ':'))


def layout_hash(episode):
    return hashlib.sha256(canonical_layout(episode).encode('utf8')).hexdigest()


def registry(suites):
    """Validate ALL supplied suites before filtering. Caller must supply full corpus."""
    environments, hashes, families, episodes, physical = {}, {}, {}, set(), set()
    for suite in suites:
        for e in suite['episodes']:
            eid, h, family, split = (e[k] for k in ('environment_id', 'layout_hash', 'layout_family_id', 'split'))
            if h != layout_hash(e):
                raise ValueError('layout_hash mismatch')
            if eid in environments and environments[eid] != (h, family, split):
                raise ValueError('environment geometry/family/split mismatch')
            if h in hashes and hashes[h] != (eid, split):
                raise ValueError('layout alias/split leakage')
            if family in families and families[family] != split:
                raise ValueError('family split leakage')
            if e['episode_id'] in episodes:
                raise ValueError('duplicate episode ID')
            # Normalize translation just as the layout hash does. Ignore IDs/seed/scenario.
            x, y = e['bounds_m'][:2]
            key = (h, e['start_pose']['x_m']-x, e['start_pose']['y_m']-y,
                   (e['start_pose']['yaw_rad'] + math.pi) % (2*math.pi)-math.pi,
                   tuple((t['label'], t['x_m']-x, t['y_m']-y, *t['size_m']) for t in e['targets']))
            if key in physical:
                raise ValueError('duplicate physical episode')
            physical.add(key)
            environments[eid], hashes[h], families[family] = (h, family, split), (eid, split), split
            episodes.add(e['episode_id'])
    return environments
