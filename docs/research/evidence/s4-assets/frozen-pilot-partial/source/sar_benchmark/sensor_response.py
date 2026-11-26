"""Counter-based synthetic depth response profiles; none is Astra calibrated."""
import hashlib
import math
import random


def depth_response(values, width, model, seed, tick, severity=1):
    if model not in ("ideal", "gaussian", "structured"):
        raise ValueError("unknown depth response model")
    if model == "ideal": return list(values)
    key = f"{seed}:depth:{model}:{tick}".encode()
    rng = random.Random(int.from_bytes(hashlib.sha256(key).digest()[:8],"big"))
    result = []
    for index,z in enumerate(values):
        if z is None:
            result.append(None); continue
        sigma = severity*(.002+.002*z*z)
        if model == "structured":
            adjacent = [index-width,index+width]
            if index % width: adjacent.append(index-1)
            if index % width < width-1: adjacent.append(index+1)
            neighbors = [values[i] for i in adjacent if 0 <= i < len(values)]
            edge = any(v is None or abs(v-z) > .10 for v in neighbors)
            probability = min(.9,.02*severity+(.4 if edge else 0)+.003*z*z*severity)
            if rng.random() < probability:
                result.append(None); continue
        bias = .006*z*severity if model == "structured" else 0.
        result.append(min(10.,max(.01,round((z+bias+rng.gauss(0,sigma))/.001)*.001)))
    return result
