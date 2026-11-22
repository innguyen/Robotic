"""Durable hash-chain journal, atomic projections and process-owned run lock."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import uuid
import time
from .artifacts import encode,digest


def atomic(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
    try:
        with temporary.open('xb') as stream:
            stream.write(data);stream.flush();os.fsync(stream.fileno())
        deadline=time.monotonic()+2
        while True:
            try:os.replace(temporary,path);break
            except PermissionError:
                if time.monotonic()>=deadline:raise
                time.sleep(.03)  # Windows readers/AV may briefly deny delete-sharing.
        if os.name!='nt':
            fd=os.open(path.parent,os.O_RDONLY)
            try:os.fsync(fd)
            finally:os.close(fd)
    finally:
        if temporary.exists():temporary.unlink()


@contextmanager
def run_lock(directory):
    path=Path(directory)/'.lock';stream=path.open('a+b')
    try:
        if stream.seek(0,2)==0:stream.write(b'0');stream.flush()
        stream.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(stream.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as error:raise ValueError('batch is owned by a live supervisor') from error
        yield
    finally:
        stream.close()  # Kernel releases ownership even on supervisor death.


class Journal:
    def __init__(self,path):
        self.path=Path(path);self.records=[];self.head='0'*64
        if not self.path.exists():return
        raw=self.path.read_bytes();offset=0
        for line in raw.splitlines(keepends=True):
            if not line.endswith(b'\n'):
                atomic(self.path.with_name('torn-'+uuid.uuid4().hex+'.bin'),raw[offset:])
                with self.path.open('r+b') as stream:stream.truncate(offset);stream.flush();os.fsync(stream.fileno())
                break
            record=json.loads(line)
            checksum=record.pop('checksum')
            if record['seq']!=len(self.records)+1 or record['previous']!=self.head or checksum!=digest(encode(record)):
                raise ValueError('journal integrity failure')
            record['checksum']=checksum;self.records.append(record);self.head=checksum;offset+=len(line)

    def append(self,kind,**payload):
        record=dict(seq=len(self.records)+1,previous=self.head,kind=kind,**payload)
        checksum=digest(encode(record));record['checksum']=checksum
        with self.path.open('ab') as stream:stream.write(encode(record)+b'\n');stream.flush();os.fsync(stream.fileno())
        self.records.append(record);self.head=checksum
        return record
