"""Launcher checks use fake SSH-free robot commands; they never contact hardware."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch
from launcher_lock_ros1 import find_supervisors


def write_unix(path, contents):
    with open(str(path), 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(contents)


class LauncherTests(unittest.TestCase):
    def test_restart_only_identifies_same_script_holding_lock(self):
        with tempfile.TemporaryDirectory(prefix='jetauto_lock_') as folder:
            root = Path(folder)
            lock = root / 'stack.lock'
            lock.write_text('')
            script = root / 'start_all_ros1.sh'
            proc = root / 'proc'
            proc.mkdir()
            for pid, command, holds_lock in (
                (100, ['/bin/bash', '-l', str(script)], True),
                (101, ['/bin/bash', str(root / 'driver.sh')], True),
                (102, ['/usr/bin/python3', str(script)], True),
                (103, ['/bin/bash', str(script)], False),
                (104, ['/bin/bash', str(script)], True),
            ):
                item = proc / str(pid)
                (item / 'fd').mkdir(parents=True)
                (item / 'cmdline').write_bytes(b'\0'.join(arg.encode() for arg in command)+b'\0')
                if holds_lock:
                    os.link(str(lock), str(item / 'fd' / '8'))
            self.assertEqual(find_supervisors(lock, script, 104, proc), [100])

    def test_stop_script_stops_only_this_stack_supervisor_first(self):
        bash = shutil.which('bash')
        git_bash = Path('C:/Program Files/Git/bin/bash.exe')
        if os.name == 'nt' and git_bash.exists():
            bash = str(git_bash)
        if not bash:
            self.skipTest('Bash not installed')
        with tempfile.TemporaryDirectory(prefix='jetauto_stop_') as folder:
            root = Path(folder)
            unix_root = str(root).replace('\\', '/')
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            write_unix(root / 'stop_test_stack_ros1.sh',
                       Path(__file__).with_name('stop_test_stack_ros1.sh').read_text(encoding='utf-8'))
            task = 'TASK'   # replaced by the folder as bash spells it (/c/... on Git Bash)
            table = [
                (201, '/bin/bash -l {}/start_all_ros1.sh live bottle 5010'.format(task)),   # old supervisor
                (202, '/bin/bash -l {}/start_server.sh --check-only'.format(task)),         # orphaned check
                (203, 'python3 {}/navigation_ros1.py'.format(task)),
                (204, 'python3 -u {}/jetauto_find_object.py person --port 5010'.format(task)),
                (205, '/usr/bin/python /opt/ros/melodic/bin/roslaunch jetauto_peripherals lidar.launch'),
                (206, 'python3 /home/jetauto/jetauto_find_object.py'),                      # not this folder
                (207, '/bin/bash -l {}/start_all_ros1.sh live bottle 5010'.format(task)),   # new caller: kept
            ]
            write_unix(root / 'table', ''.join('{} {}\n'.format(pid, args) for pid, args in table))
            write_unix(bin_dir / 'ps', '#!/bin/bash\nwhile read -r p a; do grep -qx "$p" "$TEST_DIR/killed" 2>/dev/null || echo "$p $a"; done < "$TEST_DIR/table"\n')
            write_unix(bin_dir / 'kill', '#!/bin/bash\nsig=$1; shift\nfor p in "$@"; do echo "$sig $p" >> "$TEST_DIR/signals"; echo "$p" >> "$TEST_DIR/killed"; done\n')
            write_unix(bin_dir / 'sleep', '#!/bin/bash\nexit 0\n')
            # The script's shell must use the fake kill, not the bash builtin.
            write_unix(root / 'no_builtin_kill.sh', 'enable -n kill\n')
            env = dict(os.environ, TEST_DIR=unix_root, BASH_ENV=unix_root+'/no_builtin_kill.sh')
            command = ('cd "$1"; export PATH="$PWD/bin:$PATH"; chmod +x "$1"/bin/*; '
                       'task=$(cd "$1" && pwd); sed -i "s#TASK#$task#" "$1/table"; '
                       'bash "$1/stop_test_stack_ros1.sh" 207')
            result = subprocess.run([bash, '-c', command, 'stop-test', unix_root], env=env,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    timeout=30, universal_newlines=True)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertTrue((root / 'signals').exists(), result.stdout)
            signals = (root / 'signals').read_text().split('\n')
            self.assertEqual(signals[0], '-TERM 201', result.stdout)     # supervisor first
            killed = {int(line.split()[1]) for line in signals if line}
            self.assertEqual(killed, {201, 202, 203, 204})
            self.assertNotIn('-KILL', ''.join(signals))

    def test_supervisor_autodetects_frames_and_stops_pair_on_vision_failure(self):
        bash = shutil.which('bash')
        git_bash = Path('C:/Program Files/Git/bin/bash.exe')
        if os.name == 'nt' and git_bash.exists():
            bash = str(git_bash)
        if not bash:
            self.skipTest('Bash not installed')
        with tempfile.TemporaryDirectory(prefix='jetauto_launcher_') as folder:
            root = Path(folder)
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            write_unix(root / 'start_all_ros1.sh', Path(__file__).with_name('start_all_ros1.sh').read_text(encoding='utf-8'))
            write_unix(root / 'start_server.sh', '#!/bin/bash\necho "DRIVERS $*" >> "$TEST_LOG"\n')
            write_unix(root / 'start_navigation_ros1.sh', '''#!/bin/bash
echo "PAIR $* ODOM=$JETAUTO_ODOM_FRAME BASE=$JETAUTO_BASE_FRAME" >> "$TEST_LOG"
trap 'echo PAIR_STOP >> "$TEST_LOG"; exit 0' TERM HUP
while true; do sleep .05; done
''')
            write_unix(root / 'environment.sh', '''source() { return 0; }
export -f source
''')
            for command, body in {
                'flock': 'exit 0', 'pkill': 'exit 0', 'pgrep': 'exit 1',
                'timeout': 'shift; exec "$@"',
                'python3': '''if [ "$1" = - ]; then
    input=$(cat)
    case "$input" in *wait_for_message*) echo 'laser_frame odom_test base_test';; esac
    exit 0
fi
if [ "$1" = -u ]; then
    echo "VISION $*" >> "$TEST_LOG"
    sleep .4
    exit 7
fi
exit 0''',
            }.items():
                write_unix(bin_dir / command, '#!/bin/bash\n'+body+'\n')
            env = dict(os.environ)
            unix_root = str(root).replace('\\', '/')
            env.update(BASH_ENV=unix_root+'/environment.sh', TEST_LOG=unix_root+'/events.log')
            command = ('cd "$1"; export PATH="$PWD/bin:$PATH"; chmod +x "$1"/bin/*; '
                       'bash "$1/start_all_ros1.sh" preview bottle 5010')
            result = subprocess.run([bash, '-c', command, 'launcher-test', unix_root],
                                    env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    timeout=20, universal_newlines=True)
            self.assertTrue((root / 'events.log').exists(), result.stdout)
            log = (root / 'events.log').read_text()
            self.assertEqual(result.returncode, 7, result.stdout)
            self.assertIn('DRIVERS --check-only', log)
            self.assertIn('PAIR laser_frame preview ODOM=odom_test BASE=base_test', log)
            self.assertIn('_odom_frame:=odom_test', log)
            self.assertIn('_base_frame:=base_test', log)
            self.assertIn('PAIR_STOP', log)


class WebControlTests(unittest.TestCase):
    def test_start_pause_validation_and_offline_service(self):
        modules = {'rospy': types.ModuleType('rospy')}
        for package, names in [('nav_msgs', ['Odometry', 'OccupancyGrid', 'Path']), ('sensor_msgs', ['Image', 'CameraInfo']), ('std_msgs', ['String'])]:
            pkg, msg = types.ModuleType(package), types.ModuleType(package+'.msg')
            for name in names:
                setattr(msg, name, object)
            pkg.msg = msg
            modules[package], modules[package+'.msg'] = pkg, msg
        with patch.dict(sys.modules, modules):
            path = Path(__file__).with_name('jetauto_find_object.py')
            spec = importlib.util.spec_from_file_location('vision_web_test', path)
            vision = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(vision)
        tracking = vision.TargetTracker(vision.TrackingConfig())
        self.assertEqual(vision.TARGET_CONFIDENCE['chair'], .40)
        for stamp in (1., 1.2):
            tracking.observe('chair', 0., 1., stamp)
        self.assertFalse(tracking.estimate(1.2).valid)
        tracking.observe('chair', 0., 1., 1.4)
        self.assertTrue(tracking.estimate(1.4).valid)
        # One inconsistent box is an outlier; three in a row drop the confirmed target.
        tracking.observe('chair', 1., 3., 1.5)
        self.assertTrue(tracking.estimate(1.5).valid)
        tracking.observe('chair', 1., 3., 1.6)
        tracking.observe('chair', 1., 3., 1.7)
        self.assertFalse(tracking.estimate(1.7).valid)
        calls = []
        def control(enabled):
            calls.append(enabled)
        view = vision.ObjectSeeker.__new__(vision.ObjectSeeker)
        view.state_lock, view.sensor_lock = threading.Lock(), threading.Lock()
        view.odom_frame, view.local_map, view.local_path = 'odom', None, []
        view.map_received = 0.
        view.navigation_state = {'target': [1., 2.]}
        view.odom_pose, view.odom_time = (0., 0., 0.), time.monotonic()
        message = types.SimpleNamespace(header=types.SimpleNamespace(frame_id='odom'),
            info=types.SimpleNamespace(width=2, height=2, resolution=.1,
                origin=types.SimpleNamespace(position=types.SimpleNamespace(x=-.1, y=-.1))),
            data=[-1, 0, 99, 100])
        view._callback_costmap(message)
        self.assertEqual(view.map_status()['map']['data'], [-1, 0, 99, 100])
        message.header.frame_id = 'wrong'
        message.data = [0, 0, 0, 0]
        view._callback_costmap(message)
        self.assertEqual(view.map_status()['map']['data'], [-1, 0, 99, 100])
        seeker = types.SimpleNamespace(set_navigation_enabled=control, map_status=view.map_status)
        server = vision.ThreadingHTTPServer(('127.0.0.1', 0), vision.Handler)
        server.seeker = seeker
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = 'http://127.0.0.1:'+str(server.server_port)
        def post(data):
            return urllib.request.urlopen(urllib.request.Request(url+'/navigation',
                data=json.dumps(data).encode(), headers={'Content-Type': 'application/json'}), timeout=3)
        try:
            page = urllib.request.urlopen(url, timeout=3).read().decode('utf-8')
            self.assertIn('Bắt đầu', page)
            self.assertIn('Tạm dừng', page)
            self.assertIn('Bản đồ vật cản cục bộ', page)
            map_data = json.load(urllib.request.urlopen(url+'/map', timeout=3))
            self.assertEqual(map_data['map']['data'], [-1, 0, 99, 100])
            self.assertEqual(map_data['target'], [1., 2.])
            for value in (True, False):
                self.assertTrue(json.load(post({'enabled': value}))['ok'])
            self.assertEqual(calls, [True, False])
            with self.assertRaises(urllib.error.HTTPError) as error:
                post({'enabled': 'true'})
            self.assertEqual(error.exception.code, 400)
            def offline(enabled):
                raise RuntimeError('Navigation unavailable')
            seeker.set_navigation_enabled = offline
            with self.assertRaises(urllib.error.HTTPError) as error:
                post({'enabled': True})
            self.assertEqual(error.exception.code, 503)
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
