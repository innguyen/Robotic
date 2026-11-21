#!/usr/bin/env python3
"""Bounded on-robot ROS bags, independent of laptop Wi-Fi. No motion publisher."""
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path


TOPICS = ['/scan', '/odom', '/tf', '/tf_static', '/map', '/amcl_pose',
          '/vision/target', '/jetauto_navigation/state', '/jetauto_navigation/path',
          '/jetauto_collision_monitor/state', '/collision_monitor/cmd_vel_raw',
          '/jetauto_controller/cmd_vel']


def prune_segments(directory, current_prefix, segments, anchor=None):
    """Keep the first current-session bag (latched map/TF) plus a recent ring."""
    completed = sorted((p for p in directory.glob('session_*.bag*')
                        if p.name.endswith(('.bag', '.bag.active')) and
                        not (p.name.startswith(current_prefix+'_') and p.name.endswith('.active'))),
                       key=lambda p: p.stat().st_mtime)
    if anchor is None:
        current = [p for p in completed if p.name.startswith(current_prefix+'_') and p.suffix == '.bag']
        anchor = min(current, key=lambda p: p.name) if current else None
    candidates = [p for p in completed if p != anchor]
    recent_count = max(0, segments-1-(1 if anchor is not None else 0))
    expired = candidates[:-recent_count] if recent_count else candidates
    for old in expired:
        old.unlink()
    return anchor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--segments', type=int, default=5)
    parser.add_argument('--size-mb', type=int, default=32)
    args = parser.parse_args()
    if not 2 <= args.segments <= 20 or not 8 <= args.size_mb <= 128:
        parser.error('Invalid recording limits')
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    # One bounded ring across sessions, rather than one unbounded folder per run.
    running = [True]
    child = [None]
    def stop(signum, frame):
        running[0] = False
        if child[0] is not None and child[0].poll() is None:
            child[0].send_signal(signal.SIGINT)
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, stop)
    prefix = directory / ('session_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
    command = ['rosbag', 'record', '--buffsize=16', '--split', '--size='+str(args.size_mb),
               '-O', str(prefix)] + TOPICS
    child[0] = subprocess.Popen(command)
    anchor = None
    try:
        while running[0] and child[0].poll() is None:
            # Melodic rosbag does not repeat latched maps at each split. Preserve
            # the first bag as the map/TF bootstrap while rotating recent data.
            anchor = prune_segments(directory, prefix.name, args.segments, anchor)
            time.sleep(.5)
    finally:
        if child[0].poll() is None:
            child[0].send_signal(signal.SIGINT)
        try:
            child[0].wait(timeout=15)
        except subprocess.TimeoutExpired:
            child[0].terminate()
            child[0].wait(timeout=5)
    return child[0].returncode


if __name__ == '__main__':
    raise SystemExit(main())
