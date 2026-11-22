"""Evaluator-only timed witnesses using the protocol's Euler model."""
import math
from .geometry import swept, target_box


def euler(pose, command, dt):
    x,y,yaw = pose
    vx,vy,wz = command
    unwrapped = yaw+wz*dt
    end = (x+(math.cos(yaw)*vx-math.sin(yaw)*vy)*dt,
           y+(math.sin(yaw)*vx+math.cos(yaw)*vy)*dt, unwrapped)
    return (end[0],end[1],(unwrapped+math.pi)%(2*math.pi)-math.pi), end


def check_path(poses, e, s, cfg, initial_hold=0):
    """Translate then rotate per waypoint; requested yaw differences are unwrapped.
    Every accepted tick has capped command, Euler integration and swept clearance.
    """
    start = tuple(e['start_pose'][k] for k in ('x_m','y_m','yaw_rad'))
    if type(poses) is not list or not poses:
        return dict(status='UNVERIFIED',reason='missing witness')
    if any(type(p) not in (list,tuple) or len(p)!=3 or
           any(type(v) not in (int,float) or not math.isfinite(v) for v in p) for p in poses):
        return dict(status='FAIL',reason='invalid witness pose')
    if tuple(poses[0]) != start:
        return dict(status='FAIL',reason='witness start mismatch')
    cc,dt = cfg['monitor'],s['dt_s']
    boxes = [o['bounds_m'] for o in e['obstacles']]+[target_box(t) for t in e['targets']]
    speed = min(cc[k] for k in ('max_forward','max_backward','max_lateral'))/4
    yaw_speed = cc['max_angular']/4
    state, trace, length = start, [], 0.
    if initial_hold>min(100000,round(e['time_limit_s']/dt)):
        return dict(status='UNVERIFIED',reason='initial confirmation exceeds tick budget')
    for _ in range(initial_hold):
        end,unwrapped=euler(state,(0.,0.,0.),dt)
        status=swept(state,unwrapped,cc['body_length'],cc['body_width'],e['bounds_m'],boxes,s['geometry_epsilon_m'])
        if status!='PASS':return dict(status=status,reason='initial hold sweep '+status)
        trace.append(dict(step_index=len(trace),pose=list(state),command=[0.,0.,0.],end_pose=list(end),path_length_m=0.))
        state=end
    for a,b in zip(poses,poses[1:]):
        distance = math.dist(state[:2],b[:2])
        rotation = b[2]-a[2]
        counts = (max(1,math.ceil(distance/speed/dt)), max(1,math.ceil(abs(rotation)/yaw_speed/dt)))
        if sum(counts)+len(trace) > min(100000,round(e['time_limit_s']/dt)):
            return dict(status='UNVERIFIED',reason='witness exceeds tick budget')
        n=counts[0]
        dx,dy = (b[0]-state[0])/(n*dt),(b[1]-state[1])/(n*dt)
        yaw=state[2]
        translation=(math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,0.)
        commands = [(translation,n)] if distance else []
        if rotation: commands.append(((0.,0.,rotation/(counts[1]*dt)),counts[1]))
        for command,count in commands:
            for _ in range(count):
                vx,vy,wz=command
                if not (-cc['max_backward'] <= vx <= cc['max_forward'] and abs(vy)<=cc['max_lateral'] and abs(wz)<=cc['max_angular']):
                    return dict(status='FAIL',reason='command cap violated')
                end,unwrapped=euler(state,command,dt)
                status=swept(state,unwrapped,cc['body_length'],cc['body_width'],e['bounds_m'],boxes,s['geometry_epsilon_m'])
                if status!='PASS': return dict(status=status,reason='tick sweep '+status)
                length+=math.dist(state[:2],end[:2])
                trace.append(dict(step_index=len(trace),pose=list(state),command=list(command),end_pose=list(end),path_length_m=length))
                state=end
    return dict(status='PASS',reason='Euler/caps/sweep verified',trace=trace,end_pose=list(state),ticks=len(trace),length_m=length)
