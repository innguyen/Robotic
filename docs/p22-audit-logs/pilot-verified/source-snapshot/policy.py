"""Spawned policy endpoint. Args and frames contain no GT/certificate/witness.

This is a cooperative-code boundary, not a sandbox against hostile file imports.
"""
from dataclasses import dataclass
from concurrent.futures import TimeoutError as FutureTimeout
import hashlib
import math
from pathlib import Path
import threading
import numpy as np
from collision_monitor_ros1 import Config
from navigation_core_ros1 import Navigator,NavConfig
from search_core_ros1 import SearchMission,SearchConfig
from .coverage import coverage_cells
from .selectors import (B0, B1, OFFICIAL_POLICIES, DOMAIN_VERSION, SelectionState,
                        CandidateDomain, FixedOrderSelector, GainDistanceFocusSelector)

POLICIES=('search-mission-smoke','test-idle','test-report','test-invalid-report','test-false-report',
          'test-drive','test-planner-error','test-planner-hang','test-policy-hang','test-nan',
          'test-delayed-report','test-crash-after-report') + OFFICIAL_POLICIES
MONITOR_OK=('PASS','LIMIT','SLOWDOWN','APPROACH','IDLE','STALE_COMMAND','WAIT_COMMAND')


@dataclass(frozen=True,slots=True)
class SensorFrame:
    step_index:int
    pose:tuple
    ranges:tuple
    observation:object
    monitor_state:str


class MissionAdapter(SearchMission):
    def __init__(self,nav,cfg,bounds):
        super().__init__(nav,cfg); self.bounds=bounds;self.retry_at=0.

    def choose_goal(self,pose,grid,now=None):
        rows,cols=np.indices(grid.raw.shape)
        x=(cols+grid.origin[0]+.5)*grid.resolution
        y=(rows+grid.origin[1]+.5)*grid.resolution
        cc=self.nav.cc;c,s=abs(math.cos(pose[2])),abs(math.sin(pose[2]))
        hx,hy=(c*cc.body_length+s*cc.body_width)/2,(s*cc.body_length+c*cc.body_width)/2
        a,b,aa,bb=self.bounds
        grid.allowed &= (grid.raw==0)&(x-hx>=a)&(x+hx<=aa)&(y-hy>=b)&(y+hy<=bb)
        for _ in range(grid.raw.size):
            goal=super().choose_goal(pose,grid,now)
            if goal is None or grid.pose_free(goal,pose[2]):return goal
            grid.allowed[grid.cell(goal)]=False
        return None

    def _next_goal(self,pose,now):
        command,state=super()._next_goal(pose,now)
        if state=='SEARCH_COMPLETE':
            self.state='STALLED';self.retry_at=now+self.cfg.spin_dwell_s
            return (0.,0.,0.),'POLICY_STALLED'
        return command,state

    def step(self,pose,target,standoff,now,dt):
        if self.state=='STALLED' and target is None:
            return self._next_goal(pose,now) if now>=self.retry_at else ((0.,0.,0.),'POLICY_STALLED')
        return super().step(pose,target,standoff,now,dt)


class SelectorMission(MissionAdapter):
    """One mission/lifecycle implementation for both official selectors."""
    def __init__(self, nav, cfg, bounds, selector):
        super().__init__(nav, cfg, bounds)
        self.selector = selector

    def choose_goal(self, pose, grid, now=None):
        state = SelectionState(tuple(pose), now, frozenset(self.covered),
            tuple(tuple(p) for p in self.blacklist), tuple(tuple(p) for p in self.spun_at),
            None if self.focus is None else tuple(self.focus), self.focus_until)
        domain = CandidateDomain(grid, state, self.cfg, self.bounds, (self.nav.cc.body_length, self.nav.cc.body_width))
        selected = self.selector.select(domain)
        return None if selected is None else np.asarray(selected.point, dtype=float)

    def _next_goal(self, pose, now):
        self.selector.finish('failed', self.nav_reason or 'goal_timeout_or_no_progress')
        return super()._next_goal(pose, now)

    def _start_spin(self, pose):
        self.selector.finish('reached', 'arrived_or_explore_arrive_radius')
        return super()._start_spin(pose)

    def step(self, pose, target, standoff, now, dt):
        # Match the core's temporary unreachable-target suppression while stalled.
        effective = target if now >= self.ignore_target_until else None
        if effective is not None:self.selector.finish('preempted', 'confirmed_target')
        return super().step(pose, effective, standoff, now, dt)


def policy_identity(policy_id, shared, cfg):
    """Evaluator-side provenance only. Never pass these file paths to selectors."""
    if policy_id not in OFFICIAL_POLICIES:return dict(policy_id=policy_id, version='legacy-smoke-test')
    root = Path(__file__).resolve().parent.parent
    paths = ('benchmark/selectors.py', 'benchmark/policy.py', 'benchmark/coverage.py',
             'search_core_ros1.py', 'navigation_core_ros1.py', 'collision_monitor_ros1.py')
    sources = {p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in paths}
    parameters = (dict(spacing_m=shared['b0_spacing_m'], order='chebyshev-angle-kx-ky', fallback='fixed-local-cell-order')
                  if policy_id == B0 else dict(score='gain/(distance+0.5)', focus='1+2*distance', tie='first-row-major-strict-greater',
                    min_gain_cells=cfg['search']['min_gain_cells'], camera_range=cfg['search']['camera_range']))
    return dict(policy_id=policy_id, version='2', algorithm='fixed-order-v2' if policy_id == B0 else 'gain-distance-focus-v1',
                domain_version=DOMAIN_VERSION, selector_parameters=parameters, shared_search_parameters=cfg['search'], sources=sources)


def _planner_failure():raise RuntimeError('injected planner failure')


def policy_worker(connection,public,policy_id,lifeline=None):
    """Only inputs: public configs, SensorFrame. Parent owns process cleanup."""
    from .lifecycle import watch_parent
    watch_parent(lifeline)
    nav=None
    try:
        nc=NavConfig(**{k:public['navigation'][k] for k in NavConfig.DEFAULTS})
        cc=Config(**{k:public['monitor'][k] for k in Config.DEFAULTS})
        nav=Navigator(nc,cc,async_planning=True)
        cfg = SearchConfig(**public['search'])
        selector = None
        mission = MissionAdapter(nav,cfg,tuple(public['bounds']))
        connection.send({'ready':True})
        while True:
            frame=connection.recv()
            if frame is None:break
            now=frame.step_index*public['dt_s'];pose=frame.pose;obs=frame.observation
            if policy_id in OFFICIAL_POLICIES and selector is None:
                # Anchor to the first sensed pose, not a fixture or target record.
                selector = (FixedOrderSelector(pose, public['bounds'], public['b0_spacing_m'])
                            if policy_id == B0 else GainDistanceFocusSelector())
                mission = SelectorMission(nav, cfg, tuple(public['bounds']), selector)
            nav.costmap.update(frame.ranges,-math.pi,2*math.pi/720,.05,6.,pose,now)
            nav.grid=nav.costmap.snapshot(pose,now)
            mission.covered.update(coverage_cells(nav.grid,pose,public['camera']))
            report=None;reason='TEST_IDLE';command=(0.,0.,0.)
            if policy_id=='search-mission-smoke' or policy_id in OFFICIAL_POLICIES:
                if frame.monitor_state in MONITOR_OK:
                    command,reason=mission.step(pose,None if obs is None else (obs.x_m,obs.y_m),public['standoff'],now,public['dt_s'])
                else:reason='MONITOR_'+frame.monitor_state
                if mission.found is not None and obs is not None:
                    report=dict(report_id='report-0001',label=obs.label,x_m=mission.found[0],y_m=mission.found[1],frame_id=obs.frame_id,observation_id=obs.observation_id)
            elif policy_id=='test-policy-hang':threading.Event().wait()
            elif policy_id.startswith('test-planner-'):
                nav.plan_future=nav.executor.submit(_planner_failure if policy_id=='test-planner-error' else threading.Event().wait)
            elif policy_id=='test-drive':command=(.1,0.,0.)
            elif policy_id=='test-nan':command=(float('nan'),0.,0.)
            elif policy_id in ('test-report','test-invalid-report','test-false-report','test-delayed-report','test-crash-after-report'):
                if policy_id=='test-invalid-report':report={'report_id':'invalid'}
                elif obs is not None and (policy_id!='test-delayed-report' or frame.step_index>=7):
                    report=dict(report_id='report-0001',label=obs.label,x_m=obs.x_m+(1. if policy_id=='test-false-report' else 0.),y_m=obs.y_m,frame_id=obs.frame_id,observation_id=obs.observation_id)
            barrier=nav.plan_future is not None
            if barrier:
                try:nav.plan_future.result(timeout=public['planner_wall_timeout_s'])
                except FutureTimeout:
                    if nav.plan_future.done():raise
                    connection.send({'error':'PLANNER_TIMEOUT','detail':'planner barrier deadline'})
                    break
            info=mission.info()
            response = dict(command=tuple(float(v) for v in command),report=report,mission=reason,
                            goal=info['goal'],covered_cells=len(mission.covered),planner_barrier=barrier)
            if selector is not None:
                response.update(selector_events=selector.drain(),selector_stats=dict(selector.stats(),stalled=mission.state=='STALLED'))
            connection.send(response)
    except (EOFError,BrokenPipeError):pass
    except Exception as error:
        try:connection.send({'error':'CRASH','detail':type(error).__name__+': '+str(error)})
        except (OSError,EOFError):pass
    finally:
        connection.close()
        if nav is not None:nav.close()  # May wait; supervisor must bound this process.
