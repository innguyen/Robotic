"""Identify only this launcher's verified lock-holding shell before restarting it."""
import os
from pathlib import Path
import signal
import sys


def find_supervisors(lock_path, script_path, exclude_pid, proc_root='/proc'):
    lock = os.stat(str(lock_path))
    script = os.path.realpath(str(script_path))
    found = []
    for process in Path(proc_root).iterdir():
        if not process.name.isdigit() or int(process.name) == exclude_pid:
            continue
        try:
            if process.stat().st_uid != lock.st_uid:
                continue
            fd = os.stat(str(process / 'fd' / '8'))
            if (fd.st_dev, fd.st_ino) != (lock.st_dev, lock.st_ino):
                continue
            args = [part.decode('utf-8', errors='replace')
                    for part in (process / 'cmdline').read_bytes().split(b'\0') if part]
            if not args or os.path.basename(args[0]) not in ('bash', 'sh'):
                continue
            index = 1
            while index < len(args) and args[index].startswith('-'):
                if 'c' in args[index].lstrip('-'):
                    break
                index += 1
            if index < len(args) and os.path.isabs(args[index]) and os.path.realpath(args[index]) == script:
                found.append(int(process.name))
        except (OSError, ValueError):
            continue
    return found


def main():
    lock_path, script_path, current_pid = sys.argv[1:]
    candidates = find_supervisors(lock_path, script_path, int(current_pid))
    if not candidates:
        print('Lock is held, but no matching test supervisor was verified. Nothing was stopped.')
        print('Inspect with: fuser -v ' + lock_path)
        return 1
    for pid in candidates:
        # Re-check identity and lock ownership immediately before signalling.
        if pid in find_supervisors(lock_path, script_path, int(current_pid)):
            print('Stopping previous test launcher, PID {}...'.format(pid), flush=True)
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
