"""Strict semantic replay: ignore only explicit run/attempt IDs and wall metadata."""
import argparse
import csv
import json
from pathlib import Path
from .artifacts import encode,digest


def semantic(directory):
    directory=Path(directory)
    result=json.loads((directory/'result.json').read_text(encoding='utf8'))
    result={k:v for k,v in result.items() if k not in ('run_id','attempt_id','elapsed_wall_s','workers_reaped')}
    events=[]
    for line in (directory/'events.jsonl').read_text(encoding='utf8').splitlines():
        event=json.loads(line)
        events.append({k:v for k,v in event.items() if k not in ('run_id','attempt_id')})
    with (directory/'trajectory.csv').open(encoding='utf8',newline='') as stream:trajectory=list(csv.DictReader(stream))
    return dict(result=result,events=events,trajectory=trajectory)


def compare(left,right):
    a,b=semantic(left),semantic(right)
    differences=[key for key in a if a[key]!=b[key]]
    return dict(status='PASS' if not differences else 'FAIL',differences=differences,
                left_hash=digest(encode(a)),right_hash=digest(encode(b)))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('left');parser.add_argument('right');args=parser.parse_args()
    result=compare(args.left,args.right);print(json.dumps(result));return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
