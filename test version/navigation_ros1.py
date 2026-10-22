#!/usr/bin/env python3
"""ROS 1 adapter for the lightweight Nav2-inspired local navigator.

Run collision_monitor_ros1.py first. Starts paused; enable via ~enable service.
Never publishes to a chassis topic. Requires fixed planar scan geometry and odom.
"""
import json
import math
import threading
import time
from collections import deque

from collision_monitor_ros1 import Config
from navigation_core_ros1 import NavConfig, Navigator, nearest_pose


STANDOFF = {'person': .55, 'chair': .40, 'couch': .45, 'bed': .45,
            'dining table': .40, 'tv': .50, 'refrigerator': .40}


def parse_target(data, now_ros, frame, timeout):
    """One atomic JSON message carries validity, class, pose and observation stamp."""
    if not isinstance(data, dict) or data.get('valid') is not True:
        reason = data.get('reason') if isinstance(data, dict) else None
        raise ValueError(reason if reason in ('TARGET_NEEDS_MATCHED_ODOMETRY_OR_RANGE',) else 'NO_CONFIRMED_TARGET')
    if data.get('frame_id', '').lstrip('/') != frame:
        raise ValueError('TARGET_FRAME_MISMATCH')
    stamp, x, y = (float(data[key]) for key in ('stamp', 'x', 'y'))
    if not all(math.isfinite(v) for v in (stamp, x, y)):
        raise ValueError('INVALID_TARGET')
    age = now_ros-stamp
    if stamp <= 0 or not -.05 <= age <= timeout:
        raise ValueError('STALE_TARGET')
    label = data.get('label')
    if not isinstance(label, str) or not label:
        raise ValueError('INVALID_TARGET_LABEL')
    return (x, y), label, max(0., age)


def main():
    import rospy
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, SetBoolResponse

    rospy.init_node('jetauto_navigation')
    cfg = NavConfig(**{key: rospy.get_param('~'+key, value) for key, value in NavConfig.DEFAULTS.items()})
    monitor_ns = rospy.get_param('~monitor_namespace', '/jetauto_collision_monitor').rstrip('/')
    cc = Config(**{key: rospy.get_param(monitor_ns+'/'+key, value) for key, value in Config.DEFAULTS.items()})
    frame = rospy.get_param('~odom_frame', 'odom').lstrip('/')
    base_frame = rospy.get_param('~base_frame', 'base_footprint').lstrip('/')
    scan_frame = rospy.get_param(monitor_ns+'/scan_frame', '').lstrip('/')
    scan_topic = rospy.get_param(monitor_ns+'/scan_topic', '/scan')
    output = rospy.resolve_name(rospy.get_param('~output_topic', '/collision_monitor/cmd_vel_raw'))
    monitor_input = rospy.resolve_name(rospy.get_param(monitor_ns+'/input_topic', '/collision_monitor/cmd_vel_raw'))
    if output != monitor_input or output in ('/cmd_vel', '/jetauto_controller/cmd_vel'):
        raise ValueError('Navigation output must be the Collision Monitor input, not the chassis')
    nav = Navigator(cfg, cc, async_planning=True)
    lock, stopped = threading.RLock(), threading.Event()
    history = deque(maxlen=200)
    state = dict(enabled=False, pose=None, odom_time=None, scan_time=None,
                 target=None, target_time=None, label='', monitor_time=None,
                 monitor_state='', monitor_velocity=None, goal_mode='vision', goal_error='', error='WAIT_INPUT', scan_stamp=None,
                 odom_stamp=None, target_error='WAIT_TARGET',
                 odom_error='WAIT_ODOMETRY', scan_error='WAIT_SCAN',
                 epoch=0, reset_pending=False, clear_map_pending=False,
                 pending_scan=None, scan_sequence=0, invalid_scan_sequence=0, scan_sync_s=None, pending_scan_sync_s=None)
    pub = rospy.Publisher(output, Twist, queue_size=1)
    status_pub = rospy.Publisher('~state', String, queue_size=1, latch=True)
    map_pub = rospy.Publisher('~costmap', OccupancyGrid, queue_size=1, latch=True)
    path_pub = rospy.Publisher('~path', Path, queue_size=1, latch=True)

    def enable(request):
        with lock:
            state['enabled'] = bool(request.data)
            state['epoch'] += 1
            state['reset_pending'] = True
            pub.publish(Twist())
        return SetBoolResponse(success=True, message='Enabled' if request.data else 'Paused')

    def odom_callback(msg):
        with lock:
            try:
                if msg.header.frame_id.lstrip('/') != frame or msg.child_frame_id.lstrip('/') != base_frame:
                    raise ValueError('ODOM_FRAME_MISMATCH')
                stamp = msg.header.stamp.to_sec()
                age = rospy.Time.now().to_sec()-stamp
                if stamp <= 0 or not -.05 <= age <= cfg.odom_timeout:
                    raise ValueError('STALE_ODOM_STAMP')
                if state['odom_stamp'] is not None and stamp <= state['odom_stamp']:
                    raise ValueError('NONINCREASING_ODOM_STAMP')
                q, p = msg.pose.pose.orientation, msg.pose.pose.position
                if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, p.x, p.y)):
                    raise ValueError('INVALID_ODOMETRY')
                norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                if abs(norm-1.) > .05:
                    raise ValueError('INVALID_QUATERNION')
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                pose = (p.x, p.y, yaw)
                if history:
                    previous = history[-1]
                    dt = stamp-previous[0]
                    distance = math.hypot(p.x-previous[1][0], p.y-previous[1][1])
                    turn = abs((yaw-previous[1][2]+math.pi) % (2*math.pi)-math.pi)
                    if distance > max(.20, dt*.8) or turn > max(.5, dt*2.):
                        # Odom reset/jump invalidates all remembered geometry and goals.
                        state['epoch'] += 1
                        state['reset_pending'] = state['clear_map_pending'] = True
                        state['pending_scan'] = None
                        state['scan_sequence'] += 1
                        history.clear()
                        state['scan_time'] = state['target'] = state['target_time'] = None
                        state['enabled'] = False
                        state['goal_mode'] = 'vision'
                history.append((stamp, pose))
                state.update(pose=pose, odom_time=time.monotonic()-max(0., age), odom_stamp=stamp, odom_error='')
            except (ValueError, TypeError) as exc:
                state.update(odom_time=None, error=str(exc), odom_error=str(exc))

    def scan_callback(msg):
        with lock:
            try:
                if not scan_frame or msg.header.frame_id.lstrip('/') != scan_frame:
                    raise ValueError('SCAN_FRAME_MISMATCH')
                stamp = msg.header.stamp.to_sec()
                age = rospy.Time.now().to_sec()-stamp
                if stamp <= 0 or not -.05 <= age <= cc.source_timeout:
                    raise ValueError('STALE_SCAN_STAMP')
                if state['scan_stamp'] is not None and stamp <= state['scan_stamp']:
                    raise ValueError('NONINCREASING_SCAN_STAMP')
                # Latest-only queue. Ray tracing never runs in a ROS callback or
                # while holding the lock needed by odometry/target callbacks.
                state['scan_sequence'] += 1
                state.update(pending_scan=(msg, time.monotonic()-max(0., age), state['scan_sequence']),
                             scan_stamp=stamp)
            except (ValueError, TypeError, OverflowError) as exc:
                state['scan_sequence'] += 1
                state['invalid_scan_sequence'] = state['scan_sequence']
                state.update(scan_time=None, pending_scan=None, error=str(exc), scan_error=str(exc))

    def target_callback(msg):
        with lock:
            if state['goal_mode'] == 'point':
                return
            try:
                target, label, age = parse_target(json.loads(msg.data), rospy.Time.now().to_sec(), frame, cfg.target_timeout)
                if label != state['label']:
                    state['epoch'] += 1
                    state['reset_pending'] = True
                state.update(target=target, label=label, target_time=time.monotonic()-age, target_error='')
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                had_target = state['target'] is not None
                state.update(target=None, target_time=None, target_error=str(exc))
                if had_target:
                    state['epoch'] += 1
                    state['reset_pending'] = True

    def map_goal_callback(msg):
        with lock:
            # Every goal-mode change stops the previous command immediately.
            state['enabled'] = False
            state['epoch'] += 1
            state['reset_pending'] = True
            pub.publish(Twist())
            try:
                data = json.loads(msg.data)
                if data.get('mode') == 'vision':
                    state.update(goal_mode='vision', target=None, target_time=None, goal_error='', target_error='WAIT_TARGET')
                    return
                target, _, _ = parse_target(dict(data, valid=True, label='map_point'),
                    rospy.Time.now().to_sec(), frame, 2.)
                if data.get('mode') != 'point':
                    raise ValueError('INVALID_GOAL_MODE')
                if state['scan_time'] is None or time.monotonic()-state['scan_time'] > cc.source_timeout:
                    raise ValueError('MAP_NOT_FRESH')
                if nav.grid is None or not nav.grid.point_free(target):
                    raise ValueError('GOAL_BLOCKED_OR_UNKNOWN')
                state.update(goal_mode='point', target=target, target_time=None, label='map_point',
                             target_error='', goal_error='', enabled=True)
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                state['goal_error'] = str(exc)
                state.update(goal_mode='vision', target=None, target_time=None, target_error='WAIT_TARGET')

    def monitor_callback(msg):
        with lock:
            try:
                data = json.loads(msg.data)
                status = data['state']
                if not isinstance(status, str):
                    raise ValueError('Invalid monitor state')
                state.update(monitor_state=status, monitor_time=time.monotonic(), monitor_velocity=data.get('velocity'))
            except (ValueError, KeyError, TypeError):
                state['monitor_time'] = None

    def publish_visualization(stamp, snapshot):
        fresh = (snapshot['pose'] is not None and snapshot['scan_time'] is not None
                 and snapshot['odom_time'] is not None
                 and 0 <= time.monotonic()-snapshot['scan_time'] <= cc.source_timeout
                 and 0 <= time.monotonic()-snapshot['odom_time'] <= cfg.odom_timeout)
        if fresh and (not snapshot['enabled'] or nav.grid is None):
            # Show observed space even while paused or without a YOLO target.
            nav.grid = nav.costmap.snapshot(snapshot['pose'], time.monotonic())
        if nav.grid is not None and fresh:
            msg = OccupancyGrid()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.info.resolution = cfg.resolution
            msg.info.height, msg.info.width = nav.grid.raw.shape
            msg.info.origin.position.x = float(nav.grid.origin[0]*cfg.resolution)
            msg.info.origin.position.y = float(nav.grid.origin[1]*cfg.resolution)
            msg.info.origin.orientation.w = 1.
            inflated = nav.grid.raw.copy()
            inflated[(inflated == 0) & ~nav.grid.allowed] = 98
            inflated[(nav.grid.raw == 0) & nav.grid.inflated] = 99
            msg.data = inflated.ravel().tolist()
            map_pub.publish(msg)
        path = Path()
        path.header.stamp, path.header.frame_id = stamp, frame
        for point in nav.path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(point[0]), float(point[1])
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        path_pub.publish(path)

    subscribers = [
        rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=1),
        rospy.Subscriber(scan_topic, LaserScan, scan_callback, queue_size=1),
        rospy.Subscriber(rospy.get_param('~target_topic', '/vision/target'), String, target_callback, queue_size=1),
        rospy.Subscriber('~map_goal', String, map_goal_callback, queue_size=1),
        rospy.Subscriber(monitor_ns+'/state', String, monitor_callback, queue_size=1),
    ]
    service = rospy.Service('~enable', SetBool, enable)

    def shutdown():
        stopped.set()
        with lock:
            for _ in range(3):
                pub.publish(Twist())
                time.sleep(.02)

    rospy.on_shutdown(shutdown)
    previous, last_visualization = time.monotonic(), 0.
    rospy.loginfo('ROS 1 navigation paused; enable via /jetauto_navigation/enable. Output: %s', output)

    def gate(snapshot, now):
        if not snapshot['enabled']:
            return 'PAUSED'
        for key, timeout, reason in (
                ('odom_time', cfg.odom_timeout, 'WAIT_ODOMETRY'),
                ('scan_time', cc.source_timeout, 'WAIT_SCAN'),
                ('target_time', cfg.target_timeout, 'WAIT_TARGET'),
                ('monitor_time', cc.source_timeout, 'WAIT_COLLISION_MONITOR')):
            if key == 'target_time' and snapshot['goal_mode'] == 'point':
                continue
            if snapshot[key] is None or not 0 <= now-snapshot[key] <= timeout:
                return reason
        if snapshot['monitor_state'] not in ('PASS', 'LIMIT', 'SLOWDOWN', 'APPROACH', 'IDLE', 'STALE_COMMAND', 'WAIT_COMMAND'):
            return 'MONITOR_'+snapshot['monitor_state']
        return ''

    try:
        while not rospy.is_shutdown() and not stopped.is_set():
            now = time.monotonic()
            with lock:
                reset, clear = state['reset_pending'], state['clear_map_pending']
                state['reset_pending'] = state['clear_map_pending'] = False
                pending, state['pending_scan'] = state['pending_scan'], None
                scan_epoch, history_copy = state['epoch'], list(history)
            if clear:
                nav.costmap.cells.clear()
            if reset:
                nav.reset()
            map_started = time.monotonic()
            if pending is not None:
                scan, observation_time, sequence = pending
                stamp = scan.header.stamp.to_sec()
                matched = nearest_pose(history_copy, stamp, cfg.pose_sync_tolerance)
                error = ''
                if not 0 <= time.monotonic()-observation_time <= cc.source_timeout:
                    error = 'STALE_SCAN_STAMP'
                elif matched is None:
                    error = 'SCAN_NEEDS_MATCHED_ODOMETRY'
                else:
                    try:
                        nav.costmap.update(scan.ranges, scan.angle_min, scan.angle_increment,
                            scan.range_min, scan.range_max, matched, observation_time)
                    except (ValueError, TypeError, OverflowError) as exc:
                        error = str(exc)
                with lock:
                    if scan_epoch == state['epoch'] and sequence > state['invalid_scan_sequence']:
                        state['scan_error'] = error
                        state['pending_scan_sync_s'] = min((abs(item[0]-stamp) for item in history_copy), default=None)
                        if not error:
                            state['scan_time'] = observation_time
                            state['scan_sync_s'] = min(abs(item[0]-stamp) for item in history_copy)
                        elif error == 'SCAN_NEEDS_MATCHED_ODOMETRY' and sequence == state['scan_sequence']:
                            # Scan may arrive just before its odometry. Retry while
                            # preserving the original observation age/watchdog.
                            state['pending_scan'] = pending
                        else:
                            state['scan_time'] = None
            map_ms = (time.monotonic()-map_started)*1000.
            with lock:
                snapshot = dict(state)
            plan_started = time.monotonic()
            reason = gate(snapshot, plan_started)
            velocity = (0., 0., 0.)
            if not reason:
                velocity, reason = nav.step(snapshot['pose'], snapshot['target'],
                    STANDOFF.get(snapshot['label'], cfg.default_standoff), plan_started, plan_started-previous,
                    exact_goal=snapshot['goal_mode'] == 'point')
            plan_ms = (time.monotonic()-plan_started)*1000.
            with lock:
                # A pause, odom reset, target loss, or stale source during planning
                # must prevent an old computed command being sent afterwards.
                blocked = gate(state, time.monotonic())
                if blocked or state['epoch'] != snapshot['epoch'] or stopped.is_set():
                    velocity = (0., 0., 0.)
                    reason = blocked or 'INPUT_CHANGED'
                if velocity == (0., 0., 0.) and reason not in ('STUCK', 'ARRIVED'):
                    nav.previous_velocity[:] = 0
                    if reason not in ('FOLLOW', 'FOLLOW_PARTIAL'):
                        nav.path = []
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = velocity
                pub.publish(msg)
                status_pub.publish(String(data=json.dumps(dict(state=reason, enabled=state['enabled'],
                    velocity=velocity, target=state['target'], label=state['label'],
                    goal_mode=state['goal_mode'], goal_error=state['goal_error'],
                    robot_pose=state['pose'], monitor_state=state['monitor_state'],
                    monitor_velocity=state['monitor_velocity'],
                    monitor_age_s=round(time.monotonic()-state['monitor_time'], 3) if state['monitor_time'] is not None else None,
                    limits=dict(scan_timeout_s=cc.source_timeout, odom_timeout_s=cfg.odom_timeout,
                                pose_sync_tolerance_s=cfg.pose_sync_tolerance),
                    planning=nav.planning_info,
                    odom_error=state['odom_error'], scan_error=state['scan_error'],
                    map_ms=round(map_ms, 1), plan_ms=round(plan_ms, 1),
                    async_planner_ms=round(nav.planner_ms, 1),
                    odom_age_s=round(time.monotonic()-state['odom_time'], 3) if state['odom_time'] is not None else None,
                    scan_age_s=round(time.monotonic()-state['scan_time'], 3) if state['scan_time'] is not None else None,
                    scan_sync_s=state['scan_sync_s'],
                    pending_scan_sync_s=state['pending_scan_sync_s'],
                    scan_header_age_s=round(rospy.Time.now().to_sec()-state['scan_stamp'], 3) if state['scan_stamp'] is not None else None,
                    latest_scan_odom_delta_s=round(state['scan_stamp']-state['odom_stamp'], 3) if state['scan_stamp'] is not None and state['odom_stamp'] is not None else None,
                    last_input_error=state['odom_error'] or state['scan_error'] or state['target_error'],
                    target_error=state['target_error']))))
                visualization = dict(state)
            if now-last_visualization >= .5:
                publish_visualization(rospy.Time.now(), visualization)
                last_visualization = now
            previous = now
            stopped.wait(max(0., 1./cfg.control_hz-(time.monotonic()-now)))
    finally:
        shutdown()
        nav.close()
        for subscriber in subscribers:
            subscriber.unregister()
        service.shutdown()


if __name__ == '__main__':
    main()
