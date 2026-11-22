"""Convex geometry and conservative continuous sweep. SI units, no ROS."""
import math


def footprint(pose, length, width):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x+c*a-s*b, y+s*a+c*b) for a, b in
                 ((-length/2, -width/2), (length/2, -width/2), (length/2, width/2), (-length/2, width/2)))


def box(bounds):
    x, y, xx, yy = bounds
    return ((x, y), (xx, y), (xx, yy), (x, yy))


def edges(poly):
    return zip(poly, poly[1:]+poly[:1])


def intersects(a, b):
    for poly in (a, b):
        for p, q in edges(poly):
            axis = (p[1]-q[1], q[0]-p[0])
            aa = [v[0]*axis[0]+v[1]*axis[1] for v in a]
            bb = [v[0]*axis[0]+v[1]*axis[1] for v in b]
            if max(aa) < min(bb) or max(bb) < min(aa):
                return False
    return True


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def clearance(a, b):
    if intersects(a, b):
        return 0.0
    return min(point_segment(p, u, v) for one, other in ((a,b),(b,a)) for p in one for u,v in edges(other))


def bounds_clearance(poly, bounds):
    x,y,xx,yy = bounds
    return min(min(p[0]-x, xx-p[0], p[1]-y, yy-p[1]) for p in poly)


def ray_hit(a, b, bounds):
    """Closed segment vs closed AABB; tangency occludes."""
    lo, hi = 0., 1.
    for i in range(2):
        delta = b[i]-a[i]
        if delta == 0:
            if not bounds[i] <= a[i] <= bounds[i+2]:
                return False
        else:
            t, u = sorted(((bounds[i]-a[i])/delta, (bounds[i+2]-a[i])/delta))
            lo, hi = max(lo,t), min(hi,u)
            if lo > hi:
                return False
    return True


def target_box(t):
    x,y = t['x_m'], t['y_m']
    w,h = t['size_m']
    return [x-w/2,y-h/2,x+w/2,y+h/2]


def pose_free(pose, length, width, bounds, obstacles, epsilon):
    p = footprint(pose,length,width)
    return bounds_clearance(p,bounds) >= 0 and all(clearance(p,box(o)) > epsilon for o in obstacles)


def swept(a, b, length, width, bounds, obstacles, epsilon, max_depth=20):
    """PASS=certified clear, FAIL=sampled collision/outside, else UNVERIFIED.

    b yaw must be UNWRAPPED. Hausdorff displacement bounds all intermediate
    footprint points, so a thin obstacle cannot hide between sampled endpoints.
    """
    radius = math.nextafter(math.hypot(length,width)/2,math.inf)
    visits = 0
    def visit(u,v,depth):
        nonlocal visits
        visits += 1
        if visits > 4096:
            return 'UNVERIFIED'
        mid = tuple((x+y)/2 for x,y in zip(u,v))
        poly = footprint(mid,length,width)
        bc = bounds_clearance(poly,bounds)
        cc = min((clearance(poly,box(o)) for o in obstacles), default=math.inf)
        if bc < 0 or cc <= epsilon:
            return 'FAIL'
        movement = math.nextafter(math.dist(u[:2],v[:2])/2 + radius*abs(v[2]-u[2])/2,math.inf)
        # Numerical ambiguity is not clearance evidence. This guard only makes
        # the sufficient PASS test stricter; it never permits contact/escape.
        scale=max([1.,length,width,movement]+[abs(v) for v in mid[:2]]+[abs(v) for o in obstacles for v in o])
        guard = 32*math.ulp(scale)
        if bc >= movement+guard and cc > epsilon+movement+guard:
            return 'PASS'
        if depth == max_depth:
            return 'UNVERIFIED'
        left, right = visit(u,mid,depth+1), visit(mid,v,depth+1)
        return 'FAIL' if 'FAIL' in (left,right) else ('PASS' if left == right == 'PASS' else 'UNVERIFIED')
    for p in (a,b):
        if not pose_free(p,length,width,bounds,obstacles,epsilon):
            return 'FAIL'
    return visit(a,b,0)
