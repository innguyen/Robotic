"""Private process memory samples; no sensor or policy inputs."""
import ctypes
import os
import threading
import time


def windows_memory(process):
    if os.name!="nt":return {"status":"UNAVAILABLE_NON_WINDOWS"}
    from ctypes import wintypes
    class Counters(ctypes.Structure):
        _fields_=[("cb",wintypes.DWORD),("PageFaultCount",wintypes.DWORD)]+[(name,ctypes.c_size_t) for name in
            ("PeakWorkingSetSize","WorkingSetSize","QuotaPeakPagedPoolUsage","QuotaPagedPoolUsage",
             "QuotaPeakNonPagedPoolUsage","QuotaNonPagedPoolUsage","PagefileUsage","PeakPagefileUsage","PrivateUsage")]
    counters=Counters();counters.cb=ctypes.sizeof(counters)
    function=ctypes.WinDLL("psapi",use_last_error=True).GetProcessMemoryInfo
    function.argtypes=[wintypes.HANDLE,ctypes.c_void_p,wintypes.DWORD];function.restype=wintypes.BOOL
    if not function(int(process._handle),ctypes.byref(counters),counters.cb):
        return {"status":"ERROR","winerror":ctypes.get_last_error()}
    return {"status":"MEASURED","working_set_bytes":counters.WorkingSetSize,
            "peak_working_set_bytes":counters.PeakWorkingSetSize,"private_commit_bytes":counters.PrivateUsage}


class MemorySampler:
    def __init__(self,process):
        self.process=process;self.samples=[];self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            self.samples.append({"monotonic_ns":time.monotonic_ns(),**windows_memory(self.process)})
            self.stop_event.wait(1.)

    def close(self):
        self.stop_event.set();self.thread.join(timeout=3)
        measured=[item for item in self.samples if item["status"]=="MEASURED"]
        return {"status":"MEASURED" if measured else "UNAVAILABLE","sample_interval_s":1.,
            "engine_peak_working_set_bytes":max((s["peak_working_set_bytes"] for s in measured),default=None),
            "engine_max_private_commit_bytes":max((s["private_commit_bytes"] for s in measured),default=None),
            "samples":self.samples,"scope":"UE process only; excludes policy/runner, shared GPU memory and system totals"}
