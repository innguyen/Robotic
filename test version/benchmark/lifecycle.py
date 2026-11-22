"""Spawn-safe parent lifeline. No environment or evaluator data in this channel."""
import os
import threading


def process_alive(pid):
    if os.name=='nt':
        import ctypes
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.OpenProcess.restype=ctypes.c_void_p
        kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
        kernel.CloseHandle.argtypes=[ctypes.c_void_p]
        handle=kernel.OpenProcess(0x100000,False,pid)
        if not handle:return ctypes.get_last_error()==5
        try:return kernel.WaitForSingleObject(handle,0)==258
        finally:kernel.CloseHandle(handle)
    try:
        os.kill(pid,0)
        from pathlib import Path
        stat=Path(f'/proc/{pid}/stat')
        if stat.exists() and stat.read_text().split(') ',1)[1].startswith('Z'):return False
        return True
    except ProcessLookupError:return False
    except PermissionError:return True


def watch_parent(connection):
    if connection is None:return
    def watch():
        try:
            while True:connection.recv_bytes()
        except (EOFError,OSError):
            os._exit(97)  # Includes planner threads stuck in executor shutdown.
    threading.Thread(target=watch,name='parent-lifeline',daemon=True).start()
