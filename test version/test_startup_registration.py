"""Startup must reject live controllers but allow a stale ROS Master API URI."""
import socket
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


class StartupRegistrationTests(unittest.TestCase):
    def preflight(self, endpoint=None, missing=False, publishers=None):
        script = Path(__file__).with_name('start_navigation_ros1.sh').read_text()
        code = script.split('python3 - "$MODE" <<\'PY\'\n', 1)[1].split('\nPY\n', 1)[0]
        master_error = type('MasterError', (Exception,), {})
        master = Mock()
        master.getSystemState.return_value = (publishers or [], [], [])
        master.lookupNode.return_value = 'http://127.0.0.1:12345/'
        if missing:
            master.lookupNode.side_effect = master_error('unknown node')
        rosgraph = types.SimpleNamespace(Master=Mock(return_value=master), MasterError=master_error)
        with patch.dict(sys.modules, {'rosgraph': rosgraph}), \
                patch.object(sys, 'argv', ['startup', 'live']), \
                patch.object(socket, 'setdefaulttimeout'), \
                patch('xmlrpc.client.ServerProxy', return_value=endpoint):
            exec(compile(code, 'startup_preflight', 'exec'), {})

    def test_missing_registration_allows_startup(self):
        self.preflight(missing=True)

    def test_stale_registration_connection_refused_allows_startup(self):
        endpoint = Mock()
        endpoint.getPid.side_effect = ConnectionRefusedError('old node exited')
        self.preflight(endpoint)

    def test_live_controller_still_blocks_duplicate_start(self):
        endpoint = Mock()
        endpoint.getPid.return_value = (1, '', 123)
        with self.assertRaisesRegex(SystemExit, 'Already running: /jetauto_navigation'):
            self.preflight(endpoint)

    def test_existing_chassis_publisher_still_blocks_live_mode(self):
        with self.assertRaisesRegex(SystemExit, 'Stop existing chassis publishers'):
            self.preflight(missing=True, publishers=[('/jetauto_controller/cmd_vel', ['/other'])])


if __name__ == '__main__':
    unittest.main()
