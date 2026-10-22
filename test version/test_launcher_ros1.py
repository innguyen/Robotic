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
        tracking.observe('chair', 1., 3., 1.5)
        self.assertFalse(tracking.estimate(1.5).valid)
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
