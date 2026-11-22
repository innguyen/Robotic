"""Sufficient continuous negative proofs; no failed planner/raster is evidence.

Shadow predicates and topology use Fraction of the actual binary64 input.
The robot-centre free space used for topology is an OVER-approximation of F.
Positive branch paths must separately pass the full footprint sweep.
"""
from fractions import Fraction as Q
import math
from .geometry import target_box


def exact_ray_hit(a,b,bounds):
    a,b=tuple(map(Q,a)),tuple(map(Q,b))
    bounds=tuple(map(Q,bounds)); lo,hi=Q(0),Q(1)
    for i in range(2):
        delta=b[i]-a[i]
        if not delta:
            if not bounds[i]<=a[i]<=bounds[i+2]: return False
        else:
            u,v=sorted(((bounds[i]-a[i])/delta,(bounds[i+2]-a[i])/delta))
            lo,hi=max(lo,u),min(hi,v)
            if lo>hi:return False
    return True


def shadow_covers(viewpoint, region, obstacles):
    """A convex obstacle's shadow from a point is convex (scales t>=1).
    Thus covering all four corners covers EVERY point in the rectangle.
    """
    if not all(math.isfinite(v) for v in (*viewpoint,*region)):
        return False
    x,y,xx,yy=region
    corners=((x,y),(x,yy),(xx,y),(xx,yy))
    # Prove intersection inside a slightly smaller obstacle. Tangent/borderline
    # rays are not evidence across the exact-proof / binary64-oracle boundary.
    for obstacle in obstacles:
        scale=max(1.,*(abs(v) for v in (*viewpoint,*region,*obstacle)))
        guard=Q(32*math.ulp(scale))
        a,b,c,d=map(Q,obstacle)
        if a+guard>=c-guard or b+guard>=d-guard:continue
        inset=(a+guard,b+guard,c-guard,d-guard)
        if all(exact_ray_hit(viewpoint,p,inset) for p in corners):return True
    return False


def camera_region(region,camera):
    # nextafter rounds the disc radius and AABB OUTWARD, never shrinks it.
    r=math.nextafter(math.hypot(camera['x_m'],camera['y_m']),math.inf)
    x,y,xx,yy=region
    return [math.nextafter(x-r,-math.inf),math.nextafter(y-r,-math.inf),
            math.nextafter(xx+r,math.inf),math.nextafter(yy+r,math.inf)]


def s2_occlusion(e,s):
    p=e['start_pose']; t=e['targets'][0]
    region=camera_region([p['x_m'],p['y_m'],p['x_m'],p['y_m']],s['camera'])
    return shadow_covers((t['x_m'],t['y_m']),region,[o['bounds_m'] for o in e['obstacles']])


def s4_barrier(e,s,cfg):
    t=e['targets'][0]; p=e['start_pose']
    r=math.nextafter(cfg['monitor']['body_length']/2+s['person_standoff_m']+cfg['navigation']['goal_tolerance'],math.inf)
    region=[math.nextafter(t['x_m']-r,-math.inf),math.nextafter(t['y_m']-r,-math.inf),
            math.nextafter(t['x_m']+r,math.inf),math.nextafter(t['y_m']+r,math.inf)]
    return shadow_covers((p['x_m'],p['y_m']),region,[o['bounds_m'] for o in e['obstacles']])


def branch_partition(e,s):
    gates=e.get('scenario_evidence',{}).get('gates',[])
    if len(gates)<2 or any(a[0]!=b[0] and a[1]!=b[1] for a,b in (g['segment_m'] for g in gates)):
        return None
    bounds=list(map(Q,e['bounds_m']))
    boxes=[list(map(Q,o['bounds_m'])) for o in e['obstacles']]+[list(map(Q,target_box(t))) for t in e['targets']]
    segments=[(tuple(map(Q,g['segment_m'][0])),tuple(map(Q,g['segment_m'][1]))) for g in gates]
    xs={bounds[0],bounds[2]}; ys={bounds[1],bounds[3]}
    for b in boxes:
        xs.update(v for v in (b[0],b[2]) if bounds[0]<v<bounds[2]); ys.update(v for v in (b[1],b[3]) if bounds[1]<v<bounds[3])
    for a,b in segments:
        xs.update(v for v in (a[0],b[0]) if bounds[0]<v<bounds[2]); ys.update(v for v in (a[1],b[1]) if bounds[1]<v<bounds[3])
    xs,ys=sorted(xs),sorted(ys)
    if len(xs)*len(ys)>4096:return None
    cells={}
    for i,(x,xx) in enumerate(zip(xs,xs[1:])):
        for j,(y,yy) in enumerate(zip(ys,ys[1:])):
            cx,cy=(x+xx)/2,(y+yy)/2
            if not any(b[0]<=cx<=b[2] and b[1]<=cy<=b[3] for b in boxes): cells[i,j]=(x,y,xx,yy)
    adjacency={c:[] for c in cells}
    for i,j in cells:
        for neighbor in ((i+1,j),(i,j+1)):
            if neighbor not in cells:continue
            x,y,xx,yy=cells[i,j]
            edge=((xx,y),(xx,yy)) if neighbor[0]!=i else ((x,yy),(xx,yy))
            # A shared edge on a physical closed wall is not traversable.
            mid=tuple((a+b)/2 for a,b in zip(*edge))
            if any(b[0]<=mid[0]<=b[2] and b[1]<=mid[1]<=b[3] for b in boxes):continue
            cut=[]
            for k,(a,b) in enumerate(segments):
                if (edge[0][0]==edge[1][0]==a[0]==b[0] and min(a[1],b[1])<=y and max(a[1],b[1])>=yy or
                    edge[0][1]==edge[1][1]==a[1]==b[1] and min(a[0],b[0])<=x and max(a[0],b[0])>=xx):cut.append(k)
            adjacency[i,j].append((neighbor,cut)); adjacency[neighbor].append(((i,j),cut))
    def component(seed,opened=None):
        reached={seed}; queue=[seed]
        for c in queue:
            for n,cuts in adjacency[c]:
                if (not cuts or all(k==opened for k in cuts)) and n not in reached: reached.add(n);queue.append(n)
        return reached
    remaining=set(cells); groups=[]; labels={}
    while remaining:
        group=component(min(remaining)); index=len(groups)
        for c in group:labels[c]=index
        groups.append(group);remaining-=group
    def locate(p):
        # Multiple adjoining cells are allowed, but they must share a component.
        found={labels[c] for c,b in cells.items() if b[0]<=Q(p[0])<=b[2] and b[1]<=Q(p[1])<=b[3]}
        return next(iter(found)) if len(found)==1 else None
    start=(e['start_pose']['x_m'],e['start_pose']['y_m']); base=locate(start)
    if base is None:return None
    t=e['targets'][0]; obstacles=[o['bounds_m'] for o in e['obstacles']]
    # Each exact cell is enlarged for all possible camera origins/yaws.
    for cell in groups[base]:
        region=camera_region([float(v) for v in cells[cell]],s['camera'])
        if not shadow_covers((t['x_m'],t['y_m']),region,obstacles):return None
    branches={}
    for k,g in enumerate(gates):
        opened=component(next(iter(groups[base])),k)
        linked={labels[c] for c in opened}-{base}
        if len(linked)!=1:return None
        branches[g['id']]=next(iter(linked))
    if len(set(branches.values()))<2:return None
    return dict(locate=locate,base=base,branches=branches,
                summary=dict(method='exact axis-aligned point-free overapproximation + full-footprint witnesses',
                             cells=len(cells),components=len(groups),base_component=base,gate_components=branches))
