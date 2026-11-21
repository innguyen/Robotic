"""Run closed-loop desktop acceptance scenarios and save their measured outcomes."""
import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np

from simulate_navigation import Simulation, robot_config


def exercise(name, scenario, persistent=False, goal=None, limit=3000, fault=None):
    sim = Simulation(scenario=scenario, config=robot_config(), persistent_map=persistent)
    if name == 'global_8m':
        sim.install_persistent_map((-1., -2., 10., 2.))
    if goal is not None:
        if not sim.set_goal(goal, exact=True):
            raise AssertionError(name+': goal was rejected')
    states, monitors = Counter(), Counter()
    turn_sign, reversals, largest_step = 0, 0, 0.
    collision = False
    started = time.monotonic()
    try:
        for index in range(limit):
            if fault is not None:
                fault(sim, index)
            before = sim.pose.copy()
            # Algorithm acceptance freezes simulated time while a bounded planner
            # finishes; desktop load must not create artificial mission timeouts.
            sim.step(planner_wait=5.)
            states[sim.state] += 1
            monitors[sim.monitor_state] += 1
            largest_step = max(largest_step, float(np.linalg.norm(sim.pose[:2]-before[:2])))
            sign = int(np.sign(sim.checked[2])) if abs(sim.checked[2]) > .05 else 0
            if sign and turn_sign and sign != turn_sign:
                reversals += 1
            if sign:
                turn_sign = sign
            # Independent oriented-rectangle vs world-box collision check (SAT).
            yaw = sim.pose[2]
            axes = [np.array([1., 0.]), np.array([0., 1.]),
                    np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])]
            hx, hy = sim.nav.cc.body_length/2, sim.nav.cc.body_width/2
            for box in sim.boxes:
                center = (np.asarray(box[:2])+box[2:])/2
                half = (np.asarray(box[2:])-box[:2])/2
                overlap = True
                for axis in axes:
                    robot_radius = hx*abs(axis.dot(axes[2]))+hy*abs(axis.dot(axes[3]))
                    box_radius = float(np.abs(axis).dot(half))
                    if abs(axis.dot(sim.pose[:2]-center)) >= robot_radius+box_radius-1e-6:
                        overlap = False
                        break
                collision |= overlap
            if sim.state in ('ARRIVED', 'STUCK', 'FOUND', 'SEARCH_COMPLETE'):
                break
        expected = 'FOUND' if scenario == 'search' else 'ARRIVED'
        result = dict(name=name, expected=expected, state=sim.state,
            passed=sim.state == expected and not collision, collision=bool(collision),
            simulated_seconds=round(sim.time-10., 2), wall_seconds=round(time.monotonic()-started, 3),
            position=sim.pose.tolist(), states=dict(states), monitor_states=dict(monitors),
            turn_reversals=reversals, max_translation_per_step_m=largest_step,
            plan_count=sim.nav.plan_count, path_switches=sim.nav.path_switches,
            map_memory=sim.nav.costmap.info())
        return result
    finally:
        sim.nav.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='validation_logs/navigation_upgrade_simulation.json')
    parser.add_argument('--cases', nargs='*', help='Optional subset of case names')
    args = parser.parse_args()
    cases = [('gap', 'gap', False, None), ('detour', 'detour', False, None),
             ('rear_goal', 'empty', False, (-1.2, -.3)),
             ('mapped_room', 'room', True, (2.5, 0.)),
             ('global_8m', 'empty', True, (8., 0.)),
             ('search_person', 'search', False, None),
             ('mapped_search_person', 'search', True, None)]
    results = []
    for name, scene, mapped, goal in cases:
        if args.cases and name not in args.cases:
            continue
        print('Running '+name+'...', flush=True)
        result = exercise(name, scene, mapped, goal)
        results.append(result)
        print('{}: {} ({} simulated seconds), collision={}'.format(
            name, result['state'], result['simulated_seconds'], result['collision']), flush=True)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(dict(results=results,
            limitations='Ideal 2D LiDAR/odometry and synthetic camera. No ROS, AMCL, GMapping or hardware latency.'),
            indent=2), encoding='utf-8')
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
