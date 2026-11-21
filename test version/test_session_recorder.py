"""Log retention must preserve latched map data and never remove unrelated files."""
import os
import tempfile
import unittest
from pathlib import Path

from session_recorder_ros1 import prune_segments


class RetentionTests(unittest.TestCase):
    def populate(self, directory, names):
        for index, name in enumerate(names):
            path = directory / name
            path.write_bytes(b'example')
            os.utime(str(path), (100.+index, 100.+index))

    def test_first_bag_and_active_bag_survive_while_oldest_recent_rotates(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            self.populate(directory, ['session_old_0.bag', 'session_old_1.bag.active',
                'session_current_0.bag', 'session_current_1.bag', 'session_current_2.bag',
                'session_current_3.bag', 'session_current_4.bag.active', 'user_notes.txt'])
            anchor = prune_segments(directory, 'session_current', 4)
            self.assertEqual(anchor.name, 'session_current_0.bag')
            self.assertEqual({p.name for p in directory.iterdir()}, {
                'session_current_0.bag', 'session_current_2.bag', 'session_current_3.bag',
                'session_current_4.bag.active', 'user_notes.txt'})

    def test_two_segment_limit_keeps_anchor_and_active_only(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            self.populate(directory, ['session_current_0.bag', 'session_current_1.bag',
                                       'session_current_2.bag.active'])
            prune_segments(directory, 'session_current', 2)
            self.assertEqual({p.name for p in directory.iterdir()},
                             {'session_current_0.bag', 'session_current_2.bag.active'})

    def test_before_first_split_counts_interrupted_old_recordings(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            self.populate(directory, ['session_old_0.bag.active', 'session_old_1.bag',
                                       'session_current_0.bag.active'])
            self.assertIsNone(prune_segments(directory, 'session_current', 2))
            self.assertFalse((directory / 'session_old_0.bag.active').exists())
            self.assertTrue((directory / 'session_current_0.bag.active').exists())


if __name__ == '__main__':
    unittest.main()
