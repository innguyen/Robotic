"""Host-only diagnostics. Never changes a task clock, policy or watchdog.

CPU-quiet is not proof of starvation or deadlock. Separate planner heartbeats
are unavailable in the unchanged worker; completed barriers are event evidence.
"""
import ctypes
import json
import os
from pathlib import Path
import threading
import time

from .artifacts import encode

VERSION = 'host-liveness-1'
PERIOD_S = .5
SUSPEND_GAP_S = 2.
OBSERVER_GAP_S = 5.


def clocks():
    result = dict(monotonic_s=time.monotonic(), utc_s=time.time(),
                  observer_cpu_s=time.process_time(), boot_s=None, active_s=None)
    if os.name == 'nt':
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.GetTickCount64.restype = ctypes.c_ulonglong
        result['boot_s'] = kernel.GetTickCount64()/1000.
        active = ctypes.c_ulonglong()
        if kernel.QueryUnbiasedInterruptTime(ctypes.byref(active)):
            result['active_s'] = active.value/10000000.
    elif hasattr(time, 'CLOCK_BOOTTIME'):
        result['boot_s'] = time.clock_gettime(time.CLOCK_BOOTTIME)
        result['active_s'] = time.clock_gettime(time.CLOCK_MONOTONIC)
    return result


def process_cpu(pid):
    """Cumulative kernel+user seconds; unavailable/exited/denied => null."""
    if os.name != 'nt':
        try:
            stat = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
            return (int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK')
        except (OSError, ValueError, IndexError):return None
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.GetProcessTimes.argtypes = [ctypes.c_void_p]+[ctypes.POINTER(ctypes.c_ulonglong)]*4
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = kernel.OpenProcess(0x1000, False, int(pid))
    if not handle:return None
    try:
        values = [ctypes.c_ulonglong() for _ in range(4)]
        if not kernel.GetProcessTimes(handle, *(ctypes.byref(v) for v in values)):return None
        return (values[2].value+values[3].value)/10000000.
    finally:kernel.CloseHandle(handle)


def tail(path):
    """Bounded read; never repair a partial writer record."""
    try:
        with path.open('rb') as stream:
            stream.seek(0, 2); size = stream.tell(); start = max(0, size-8192)
            stream.seek(start); raw = stream.read()
        lines = raw.split(b'\n')[:-1]
        if start:lines = lines[1:]
        return lines[-1].decode('utf8') if lines else None
    except FileNotFoundError:return None


def progress(slot):
    folder = slot/'attempts'
    candidates = sorted(folder.glob('*/attempt-*')) if folder.exists() else []
    if not candidates:return dict(step_index=None,sim_time_s=None,workers={},event=None)
    attempt = candidates[-1]
    pids = json.loads((attempt/'workers.json').read_bytes()) if (attempt/'workers.json').exists() else {}
    row = tail(attempt/'trajectory.csv'); event = tail(attempt/'events.jsonl')
    step = sim = None
    if row and not row.startswith('step_index'):
        fields = row.split(',');step = int(fields[0]);sim = float(fields[1])
    record = json.loads(event) if event else None
    return dict(step_index=step,sim_time_s=sim,workers=pids,
        event=None if record is None else {k:record[k] for k in ('event_seq','event_type','step_index','sim_time_s')})


def review(samples):
    issues = []; gaps = []; utc_jumps = []; no_tick = []
    if len(samples)<2:issues.append('insufficient host samples')
    for i, sample in enumerate(samples):
        if sample['seq']!=i+1:issues.append('host sample sequence mismatch')
        if sample.get('error'):issues.append('host observation error')
        if sample['clock']['active_s'] is None or sample['clock']['boot_s'] is None:
            issues.append('host active/boot clock unavailable')
    for a,b in zip(samples,samples[1:]):
        da = b['clock']['monotonic_s']-a['clock']['monotonic_s']
        if da<0:issues.append('monotonic clock reversed')
        if da>OBSERVER_GAP_S:issues.append('host observer sampling gap')
        if abs(b['clock']['utc_s']-a['clock']['utc_s']-da)>SUSPEND_GAP_S:utc_jumps.append(b['seq'])
        active = (a['clock']['active_s'],b['clock']['active_s'],a['clock']['boot_s'],b['clock']['boot_s'])
        if None not in active:
            suspend = (active[3]-active[2])-(active[1]-active[0])
            if suspend>SUSPEND_GAP_S:gaps.append(dict(seq=b['seq'],unbiased_gap_s=suspend))
        if a['progress']['step_index'] is not None and a['progress']['step_index']==b['progress']['step_index']:
            cpu = [(a['worker_cpu_s'].get(k),v) for k,v in b['worker_cpu_s'].items()]
            busy = any(x is not None and y is not None and y>x for x,y in cpu)
            no_tick.append(dict(seq=b['seq'],observation='CPU_ACTIVE_NO_TICK' if busy else 'CPU_QUIET_OR_UNAVAILABLE_NO_TICK'))
    if gaps:issues.append('boot/active clocks indicate host suspend gap')
    return dict(version=VERSION,status='PASS' if not issues else 'UNVERIFIED',issues=sorted(set(issues)),
        samples=len(samples),suspend_gaps=gaps,utc_jump_sequences=utc_jumps,no_tick_observations=no_tick,
        planner_heartbeat='UNVERIFIED: no separate worker/planner phase channel',
        causal_limit='CPU-quiet cannot distinguish starvation, blocking or deadlock; CPU-active can be expensive work or spinning.')


class Recorder:
    def __init__(self,directory,slot):
        self.directory=Path(directory);self.slot=Path(slot);self.samples=[];self.stop_event=threading.Event()
        self.thread=None;self.thread_error=None

    def sample(self):
        row=dict(seq=len(self.samples)+1,clock=clocks(),progress=dict(step_index=None,sim_time_s=None,workers={},event=None),worker_cpu_s={})
        if self.thread_error:row['error']=self.thread_error
        try:
            row['progress']=progress(self.slot)
            row['worker_cpu_s']={k:process_cpu(pid) for k,pid in row['progress']['workers'].items()}
        except (OSError,ValueError,KeyError) as error:row['error']=type(error).__name__
        self.stream.write(encode(row)+b'\n');self.stream.flush();self.samples.append(row)

    def __enter__(self):
        self.directory.mkdir(parents=True,exist_ok=False)
        self.stream=(self.directory/'samples.jsonl').open('xb');self.sample()
        def observe():
            while not self.stop_event.wait(PERIOD_S):
                try:self.sample()
                except Exception as error:
                    self.thread_error=type(error).__name__;break
        self.thread=threading.Thread(target=observe,name='host-liveness',daemon=True);self.thread.start()
        return self

    def __exit__(self,*exc):
        self.stop_event.set();self.thread.join(timeout=2.)
        if self.thread.is_alive():raise RuntimeError('host observer did not stop; diagnostics incomplete')
        self.sample();os.fsync(self.stream.fileno());self.stream.close()
        self.summary=review(self.samples)
