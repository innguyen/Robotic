"""Spawned policy endpoint. Args and frames contain no GT/certificate/witness.

This is a cooperative-code boundary, not a sandbox against hostile file imports.
"""
from dataclasses import dataclass
from concurrent.futures import TimeoutError as FutureTimeout
import math
import threading
import numpy as np
from collision_monitor_ros1 import Config
from navigation_core_ros1 import Navigator,NavConfig
from search_core_ros1 import SearchMission,SearchConfig
from .coverage import coverage_cells

POLICIES=('search-mission-smoke','test-idle','test-report','test-invalid-report','test-false-report',
          'test-drive','test-planner-error','test-planner-hang','test-policy-hang','test-nan',
          'test-delayed-report','test-crash-after-report')
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


def _planner_failure():raise RuntimeError('injected planner failure')


def policy_worker(connection,public,policy_id):
    """Only inputs: public configs, SensorFrame. Parent owns process cleanup."""
    nav=None
    try:
        nc=NavConfig(**{k:public['navigation'][k] for k in NavConfig.DEFAULTS})
        cc=Config(**{k:public['monitor'][k] for k in Config.DEFAULTS})
        nav=Navigator(nc,cc,async_planning=True)
        mission=MissionAdapter(nav,SearchConfig(**public['search']),tuple(public['bounds']))
        connection.send({'ready':True})
        while True:
            frame=connection.recv()
            if frame is None:break
            now=frame.step_index*public['dt_s'];pose=frame.pose;obs=frame.observation
            nav.costmap.update(frame.ranges,-math.pi,2*math.pi/720,.05,6.,pose,now)
            nav.grid=nav.costmap.snapshot(pose,now)
            mission.covered.update(coverage_cells(nav.grid,pose,public['camera']))
            report=None;reason='TEST_IDLE';command=(0.,0.,0.)
            if policy_id=='search-mission-smoke':
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
            connection.send(dict(command=tuple(float(v) for v in command),report=report,mission=reason,
                                 goal=info['goal'],covered_cells=len(mission.covered),planner_barrier=barrier))
    except (EOFError,BrokenPipeError):pass
    except Exception as error:
        try:connection.send({'error':'CRASH','detail':type(error).__name__+': '+str(error)})
        except (OSError,EOFError):pass
    finally:
        connection.close()
        if nav is not None:nav.close()  # May wait; supervisor must bound this process.
