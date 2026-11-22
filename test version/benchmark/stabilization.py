"""Versioned, opt-in host observation around unchanged paired/batch execution."""
import argparse
import json
import platform
import time
from pathlib import Path

from . import paired
from .artifacts import encode,digest,output_path
from .schema import load_json
from .journal import atomic,run_lock
from .host_liveness import Recorder,VERSION as HOST_VERSION,PERIOD_S,SUSPEND_GAP_S,OBSERVER_GAP_S,review

VERSION='stabilized-paired-1'


def contract(output):
    directory=output_path(output)
    return dict(version=VERSION,paired_manifest_hash=digest((directory/'paired_manifest.json').read_bytes()),
        host_version=HOST_VERSION,period_s=PERIOD_S,suspend_gap_s=SUSPEND_GAP_S,observer_gap_s=OBSERVER_GAP_S,
        clock=time.get_clock_info('monotonic').__dict__,platform=platform.platform(),
        sources={name:digest((Path(__file__).parent/name).read_bytes()) for name in ('host_liveness.py','stabilization.py')},
        power_settings_changed=False,watchdog_changed=False,scoring_changed=False,concurrency=1,
        semantics='sidecar only; no diagnostics enter policy, evaluator scoring, or semantic replay')


def initialize(output):
    directory=output_path(output)
    with run_lock(directory):
        paired.load_pair(output)
        if (directory/'stabilization_manifest.json').exists() or (directory/'runs').exists():raise ValueError('fresh paired run required')
        atomic(directory/'stabilization_manifest.json',encode(contract(output)))


def locked_contract(output):
    actual=load_json(output_path(output)/'stabilization_manifest.json')
    if actual!=contract(output):raise ValueError('stabilization identity changed')
    return actual


def job(output,policy,replica,eid):
    directory=output_path(output)
    with run_lock(directory):
        identity=locked_contract(output);spec=paired.load_pair(output)
        if policy not in paired.POLICIES or replica not in paired.REPLICAS or eid not in spec['selection']:raise ValueError('unknown stabilized slot')
        slot=output_path(spec['slots'][replica][policy][eid])
        host=directory/'host'/replica/policy/eid
        if slot.exists() or host.exists():raise ValueError('attempt 1 only; existing slot/host evidence must be preserved')
        with Recorder(host,slot) as recorder:
            summary=paired._job_locked(output,policy,replica,eid)
        records,head=paired.journal_records(slot/'journal.jsonl')
        report=dict(version=VERSION,identity_hash=digest(encode(identity)),pair_hash=digest(encode(spec)),
            policy=policy,replica=replica,episode_id=eid,slot=slot.relative_to(paired.ROOT).as_posix(),
            journal_head=head,samples_hash=digest((host/'samples.jsonl').read_bytes()),diagnostics=recorder.summary)
        atomic(host/'result.json',encode(report))
        return dict(status='PASS' if recorder.summary['status']=='PASS' and not summary['overall']['execution_errors'] else 'FAIL',
                    execution_errors=summary['overall']['execution_errors'],host=recorder.summary['status'])


def audit(output):
    directory=output_path(output)
    with run_lock(directory):
        identity=locked_contract(output);spec=paired.load_pair(output);issues=[];results=[]
        for replica in paired.REPLICAS:
            for policy in paired.POLICIES:
                for eid in spec['selection']:
                    folder=directory/'host'/replica/policy/eid;report=load_json(folder/'result.json')
                    slot=spec['slots'][replica][policy][eid];_,head=paired.journal_records(output_path(slot)/'journal.jsonl')
                    raw=(folder/'samples.jsonl').read_bytes()
                    if not raw.endswith(b'\n'):raise ValueError('torn host samples')
                    samples=[json.loads(line) for line in raw.splitlines()]
                    if (report['identity_hash']!=digest(encode(identity)) or report['pair_hash']!=digest(encode(spec)) or
                        report['slot']!=slot or report['policy']!=policy or report['replica']!=replica or report['episode_id']!=eid or
                        report['journal_head']!=head or report['samples_hash']!=digest(raw) or report['diagnostics']!=review(samples)):
                        raise ValueError('host evidence identity/hash mismatch')
                    if report['diagnostics']['status']!='PASS':issues.append('host liveness unverified: '+replica+'/'+policy+'/'+eid)
                    final=load_json(output_path(slot)/f'attempts/{eid}/attempt-0001/result.json')
                    if samples[-1]['progress']['sim_time_s']!=final['elapsed_sim_s']:issues.append('host final tick mismatch: '+slot)
                    results.append(report)
        result=paired._audit_locked(output)
        if any(r['metrics']['execution_error'] for r in result['reductions']):issues.append('execution error retained')
        report=dict(version=VERSION,status='PASS' if result['status']=='PASS' and not issues else 'FAIL',
            issues=issues,paired_status=result['status'],host_jobs=results,identity=identity)
        atomic(directory/'analysis/stabilization-audit.json',encode(report));return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('init','run','audit'))
    parser.add_argument('--output',required=True);parser.add_argument('--policy');parser.add_argument('--replica');parser.add_argument('--episode-id')
    args=parser.parse_args()
    try:
        if args.action=='init':initialize(args.output);result=dict(status='INITIALIZED')
        elif args.action=='run':result=job(args.output,args.policy,args.replica,args.episode_id)
        else:
            report=audit(args.output);result={k:report[k] for k in ('status','issues','paired_status')}
        print(json.dumps(result),flush=True);return int(result['status']=='FAIL')
    except (ValueError,OSError,KeyError,TypeError,RuntimeError) as error:print(json.dumps(dict(status='BLOCKED',error=str(error))));return 2


if __name__=='__main__':raise SystemExit(main())
