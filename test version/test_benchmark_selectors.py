"""P2.1 selectors, production compatibility, shared lifecycle and spawn boundary."""
import copy
import csv
from dataclasses import FrozenInstanceError, fields
import json
import math
import multiprocessing as mp
import threading
import unittest
from unittest.mock import patch

import numpy as np
from collision_monitor_ros1 import Config
from navigation_core_ros1 import Grid, Navigator, NavConfig
from search_core_ros1 import SearchConfig, SearchMission
from benchmark.selectors import (B0, B1, CandidateDomain, SelectionState,
    FixedOrderSelector, GainDistanceFocusSelector, selection_diagnostics)
from benchmark.policy import SelectorMission, SensorFrame, policy_worker, policy_identity
from benchmark.runner import run_single
from benchmark.batch import run_batch
from benchmark.replay import compare
from benchmark.schema import ROOT, load_json
from benchmark.simulator import lidar, world_boxes
from benchmark.oracle import Oracle
import test_benchmark_runner as single_tests


def never_ready(connection, public, policy_id, lifeline=None):
    """Test-only bootstrap hang; the supervisor must reap it."""
    threading.Event().wait()


def state(pose=(0., 0., 0.), **kwargs):
    return SelectionState(pose, 0., **dict(dict(covered=frozenset(), blacklist=(), spun_at=(), focus=None, focus_until=0.), **kwargs))


def grid():
    raw = np.zeros((16, 16), dtype=np.int8)
    return Grid(raw, raw == 0, (-8, -8), .25)


def domain(g=None, s=None, cfg=None, bounds=(-2., -2., 2., 2.)):
    return CandidateDomain(grid() if g is None else g, state() if s is None else s,
                           cfg or SearchConfig(min_goal_distance=.5, min_gain_cells=1., camera_range=.75), bounds, (.324, .25))


class SelectorTests(unittest.TestCase):
    def test_ring_order_and_start_rotation(self):
        selector = FixedOrderSelector((0., 0., 0.), (-1., -1., 1., 1.), .75)
        expected = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
        self.assertEqual([p[:2] for p in selector.waypoints], expected)
        rotated = FixedOrderSelector((3., -2., math.pi/2), (1., -4., 5., 0.), .75)
        np.testing.assert_allclose(rotated.waypoints[0][2], (3., -1.25), atol=1e-15)
        self.assertFalse(any((kx, ky) == (0, 0) for kx, ky, _ in rotated.waypoints))

    def test_fixed_point_is_not_snapped_to_cell_centre(self):
        selected = FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), .75).select(domain())
        self.assertEqual(selected.point, (.75, 0.))
        self.assertEqual((selected.kind, selected.waypoint_index), ('fixed', 0))

    def test_unknown_and_outside_window_are_reconsidered(self):
        for mode in ('unknown', 'outside'):
            with self.subTest(mode=mode):
                selector = FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), .75)
                g = grid()
                if mode == 'unknown':g.raw[g.cell((.75, 0.))] = -1
                else:g.origin = np.array((-24, -8))
                selector.select(domain(g))
                self.assertNotIn(0, selector.assigned)
                self.assertTrue(any(e[1].get('skipped') for e in selector.drain()))
                selected = selector.select(domain())
                self.assertEqual(selected.waypoint_index, 0)

    def test_assigned_waypoint_never_reissued_even_after_failure(self):
        selector = FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), .75)
        first = selector.select(domain())
        selector.finish('failed', 'STUCK')
        second = selector.select(domain())
        self.assertNotEqual(first.waypoint_index, second.waypoint_index)
        outcomes = [p for kind, p in selector.drain() if kind == 'WAYPOINT_OUTCOME']
        self.assertEqual(outcomes[0]['outcome'], 'failed')
        self.assertEqual(outcomes[0]['waypoint_index'], 0)
        self.assertEqual(selector.stats()['unresolved_waypoints'], len(selector.waypoints))

    def test_fallback_fixed_order_ignores_gain_and_focus(self):
        cfg = SearchConfig(min_goal_distance=.5, min_gain_cells=1.e9)
        d = domain(s=state(focus=(100., -100.), focus_until=100.), cfg=cfg)
        selector = FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), 10.)
        selected = selector.select(d)
        self.assertEqual(selected.kind, 'fallback')
        self.assertEqual(selected.point, (.375, .375))
        self.assertEqual(selector.stats()['fallback_goals'], 1)
        self.assertIsNone(GainDistanceFocusSelector().select(d))

    def test_common_blacklist_revisit_distance_bounds_and_footprint(self):
        g = grid(); cfg = SearchConfig(min_goal_distance=.5, camera_range=.75)
        d = domain(g, state(blacklist=((.75, 0.),), spun_at=((0., .75),)), cfg)
        self.assertEqual(d.rejection((.75, 0.)), 'blacklist')
        self.assertEqual(d.rejection((0., .75)), 'revisit')
        self.assertEqual(d.rejection((.1, .1)), 'min_distance')
        self.assertIsNotNone(d.rejection((1.95, 0.)))
        before = g.allowed.copy()
        g.pose_free = lambda point, yaw:point[0] < 0
        for selector in (FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), .75), GainDistanceFocusSelector()):
            selected = selector.select(d)
            self.assertIsNotNone(selected)
            self.assertLess(selected.point[0], 0.)
            self.assertIsNone(d.rejection(selected.point))
        np.testing.assert_array_equal(g.allowed, before)

    def test_b1_matches_production_score_focus_threshold_and_ties(self):
        # Bounds contain the whole map; pose_free=True. Thus the only changes
        # made by §5.2 are inactive and exact equality with production is required.
        rng = np.random.default_rng(918)
        for seed in range(24):
            raw = rng.choice([-1, 0, 100], (17, 17), p=[.1, .85, .05]).astype(np.int8)
            g = Grid(raw, raw == 0, (-8, -8), .25)
            cfg = SearchConfig(min_gain_cells=1. if seed % 3 else 1.e6, camera_range=.75)
            nav = Navigator(); mission = SearchMission(nav, cfg)
            mission.covered = {(i, j) for i in range(-8, 9) for j in range(-8, 9) if (i+j+seed) % 3}
            mission.blacklist = [np.array((.6, .6))]; mission.spun_at = [np.array((-.6, -.6))]
            mission.focus = np.array((1., -.5)) if seed % 2 else None; mission.focus_until = 3.
            s = SelectionState((0., 0., .2), float(seed % 5), frozenset(mission.covered),
                               ((.6, .6),), ((-.6, -.6),), None if mission.focus is None else tuple(mission.focus), 3.)
            expected = mission.choose_goal(s.pose, g, s.now)
            actual = GainDistanceFocusSelector().select(domain(g, s, cfg, (-10., -10., 10., 10.)))
            if expected is None:self.assertIsNone(actual)
            else:np.testing.assert_array_equal(actual.point, expected)

    def test_state_has_no_gt_or_mission_reference_and_is_immutable(self):
        self.assertEqual({f.name for f in fields(SelectionState)}, {'pose','now','covered','blacklist','spun_at','focus','focus_until'})
        with self.assertRaises(FrozenInstanceError):state().now = 1.
        with self.assertRaises(TypeError):SelectionState(targets=())

    def test_b1_optional_now_preserves_production_focus_semantics(self):
        g = grid(); cfg = SearchConfig(min_gain_cells=1., camera_range=.75)
        mission = SearchMission(Navigator(), cfg); mission.focus = np.array((1., 1.)); mission.focus_until = 10.
        observed = SelectionState((0.,0.,0.), None, frozenset(), (), (), (1.,1.), 10.)
        expected = mission.choose_goal(observed.pose, g, None)
        actual = GainDistanceFocusSelector().select(domain(g, observed, cfg))
        np.testing.assert_array_equal(actual.point, expected)

    def test_stalled_retry_and_new_target_enter_approach(self):
        for selector in (FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), .75), GainDistanceFocusSelector()):
            nav = Navigator(); nav.grid = grid()
            nav.grid.raw[:] = -1; nav.grid.allowed[:] = False
            mission = SelectorMission(nav, SearchConfig(spin_dwell_s=.5), (-2., -2., 2., 2.), selector)
            self.assertEqual(mission._next_goal((0., 0., 0.), 0.)[1], 'POLICY_STALLED')
            selector.drain()
            self.assertEqual(mission.step((0., 0., 0.), None, .55, .4, .1)[1], 'POLICY_STALLED')
            self.assertEqual(selector.drain(), [])
            mission.step((0., 0., 0.), None, .55, .5, .1)
            self.assertTrue(selector.drain())
            nav.step = lambda *args, **kwargs:((.02, 0., 0.), 'FOLLOW')
            command, _ = mission.step((0., 0., 0.), (1., 0.), .55, .6, .1)
            self.assertEqual(mission.state, 'APPROACH'); self.assertGreater(command[0], 0.)
            mission.ignore_target_until = 10.; mission.state = 'STALLED'
            self.assertEqual(mission.step((0., 0., 0.), (1., 0.), .55, .7, .1)[1], 'POLICY_STALLED')

    def test_stalled_retry_can_select_after_map_update(self):
        for selector in (FixedOrderSelector((0., 0., 0.), (-2., -2., 2., 2.), .75), GainDistanceFocusSelector()):
            nav = Navigator(); nav.grid = grid(); nav.grid.raw[:] = -1; nav.grid.allowed[:] = False
            mission = SelectorMission(nav, SearchConfig(spin_dwell_s=.5), (-2., -2., 2., 2.), selector)
            mission._next_goal((0., 0., 0.), 0.); nav.grid = grid()
            _, reason = mission.step((0., 0., 0.), None, .55, .5, .1)
            self.assertEqual(reason, 'SEARCH_EXPLORE'); self.assertEqual(mission.state, 'GOTO')

    def test_diagnostics_continuous_stall_and_waypoint_outcomes(self):
        records = [dict(event_type='SELECTOR_STATUS', sim_time_s=t, payload=dict(stalled=stalled,
            goals_selected=count, fallback_goals=1, unresolved_waypoints=8)) for t, stalled, count in [(0,False,0),(1,True,0),(2,True,0),(3,False,1),(4,True,1)]]
        result = selection_diagnostics(records, 7.)
        self.assertEqual(result['first_stalled_s'], 1.); self.assertEqual(result['stalled_sim_s'], 5.)
        self.assertEqual(result['goals_selected'], 1)


class OfficialRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        single_tests.RunnerTests.setUpClass(); cls.base = single_tests.RunnerTests; cls.counter = 0

    @classmethod
    def tearDownClass(cls):single_tests.RunnerTests.tearDownClass()

    def folder(self):
        type(self).counter += 1
        return self.base.relative+'/p21-'+str(self.counter)

    def test_official_policies_full_single_loop_and_replay(self):
        for policy in (B0, B1):
            a, b = self.folder(), self.folder()
            for folder in (a, b):
                result = run_single(self.base.catalog, self.base.bundle, 'positive', folder, policy)
                self.assertEqual(result['end_reason'], 'SUCCESS_REPORT'); self.assertTrue(result['workers_reaped'])
                self.assertEqual(result['goals_selected'], 0)  # Visible immediately: approach supersedes exploration.
                manifest = load_json(ROOT/folder/'run_manifest.json')
                self.assertEqual(manifest['policy_identity']['policy_id'], policy)
                self.assertIn('benchmark/selectors.py', manifest['policy_identity']['sources'])
                records = [json.loads(line) for line in (ROOT/folder/'events.jsonl').read_text().splitlines()]
                self.assertTrue(any(e['event_type']=='SELECTOR_STATUS' for e in records))
                self.assertTrue(all(e['source']=='policy' for e in records if e['event_type']=='SELECTOR_STATUS'))
            self.assertEqual(compare(ROOT/a, ROOT/b)['status'], 'PASS')
            manifest = load_json(ROOT/b/'run_manifest.json'); manifest['policy_identity']['version'] = 'changed'
            (ROOT/b/'run_manifest.json').write_text(json.dumps(manifest), encoding='utf8')
            self.assertIn('policy', compare(ROOT/a, ROOT/b)['differences'])

    def test_batch_resume_rejects_selector_parameters_or_source_identity_change(self):
        for policy in (B0, B1):
            folder = self.folder()
            run_batch(self.base.catalog, self.base.bundle, folder, policy, episode_ids=['empty'])
            original = (ROOT/folder/'journal.jsonl').read_bytes()
            run_batch(self.base.catalog, self.base.bundle, folder, policy, episode_ids=['empty'], resume=True)
            self.assertEqual(original, (ROOT/folder/'journal.jsonl').read_bytes())
            for changed in ('selector_parameters', 'sources'):
                from benchmark.policy import policy_identity as real
                def altered(*args):
                    value = real(*args); value[changed] = dict(changed='invalid'); return value
                with patch('benchmark.batch.policy_identity', side_effect=altered):
                    with self.assertRaisesRegex(ValueError, 'fingerprints'):
                        run_batch(self.base.catalog, self.base.bundle, folder, policy, episode_ids=['empty'], resume=True)

    def test_bootstrap_failure_keeps_unknown_selector_counters_null(self):
        for policy in (B0, B1):
            folder = self.folder()
            with patch('benchmark.runner.policy_worker', never_ready):
                run_batch(self.base.catalog, self.base.bundle, folder, policy, episode_ids=['hang'])
            result = load_json(ROOT/folder/'attempts/hang/attempt-0001/result.json')
            self.assertEqual(result['end_reason'], 'WALL_TIMEOUT'); self.assertTrue(result['workers_reaped'])
            for name in ('goals_selected','fallback_goals','unresolved_waypoints','stalled_sim_s'):
                self.assertIsNone(result[name])
            with (ROOT/folder/'episodes.csv').open(newline='') as stream:row = next(csv.DictReader(stream))
            self.assertEqual(row['goals_selected'], ''); self.assertEqual(row['stalled_sim_s'], '')

    def test_spawned_policy_same_observed_inputs_despite_different_hidden_worlds(self):
        # Two worlds differ only behind an opaque wall. Generate actual LiDAR and
        # oracle frames independently; worker gets only their identical sensors.
        template = load_json(ROOT/'test version/benchmark/fixtures/samples.json')
        public_cfg = __import__('benchmark.schema', fromlist=['configs']).configs(template['shared'], ROOT)
        public = {k:public_cfg[k] for k in ('navigation','monitor','search')}
        public['search'] = dict(public['search'], spin_step_deg=360., spin_dwell_s=.1)
        public.update(bounds=(-2.,-2.,2.,2.), camera=template['shared']['camera'], dt_s=.1, standoff=.55,
                      b0_spacing_m=.75, planner_wall_timeout_s=5.)
        frames = []
        for x in (2., 4.):
            e = copy.deepcopy(template['episodes'][0]); e['targets'][0]['x_m'] = x; e['targets'][0]['id'] = 'hidden-'+str(x)
            e['obstacles'] = [dict(id='opaque', bounds_m=[1.,-10.,1.2,10.])]
            cc = Config(**{k:public['monitor'][k] for k in Config.DEFAULTS})
            ranges = lidar((0.,0.,0.), world_boxes(e), cc)
            oracle = Oracle(public['camera'], .1)
            frames.append([SensorFrame(k,(0.,0.,0.),ranges,oracle.step(k,(0.,0.,0.),e['targets'][0],e['obstacles'])[1],'IDLE') for k in range(4)])
        self.assertEqual(frames[0], frames[1])
        for policy in (B0, B1):
            outputs = []
            for series in frames:
                ctx = mp.get_context('spawn'); parent, child = ctx.Pipe()
                process = ctx.Process(target=policy_worker, args=(child,public,policy))
                process.start(); child.close()
                try:
                    self.assertTrue(parent.poll(10.)); self.assertTrue(parent.recv()['ready'])
                    responses = []
                    for frame in series:
                        parent.send(frame); self.assertTrue(parent.poll(10.)); response = parent.recv()
                        self.assertNotIn('error',response); responses.append(response)
                    outputs.append(responses)
                    parent.send(None); process.join(10.); self.assertEqual(process.exitcode, 0)
                finally:
                    if process.is_alive():process.kill(); process.join(3.)
                    parent.close()
            self.assertEqual(outputs[0], outputs[1])
            self.assertTrue(any(r['selector_stats']['goals_selected'] for r in outputs[0]))


if __name__ == '__main__':unittest.main()
