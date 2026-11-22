"""Run certified dev controls twice per official policy; never select test episodes.

From repository root: python -B docs/checks/p21-dev-controls.py --output docs/p21-audit-logs/dev
This is control/replay evidence, not a baseline comparison or research evaluation.
"""
import argparse
import json
import platform
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'test version'))
from benchmark.artifacts import load_catalog, build_bundle, persist, output_path, encode
from benchmark.batch import run_batch
from benchmark.replay import compare
from benchmark.selectors import OFFICIAL_POLICIES
from benchmark.journal import atomic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--finalize-existing', action='store_true', help='Audit complete existing batches; never fill missing attempts')
    parser.add_argument('--b1-repeat', choices=('run-2','run-3'), default='run-2', help='Repeat batch retained after a supervisor interruption')
    args = parser.parse_args()
    destination = output_path(args.output)
    if destination.exists() and not args.finalize_existing:raise ValueError('control output already exists; use a new directory')
    if args.finalize_existing and not destination.is_dir():raise ValueError('existing controls missing')
    if args.b1_repeat != 'run-2' and not args.finalize_existing:raise ValueError('alternate repeat requires finalize-existing')
    catalog_path = 'test version/benchmark/fixtures/catalog.json'
    catalog, suites, cfgs, witnesses = load_catalog(catalog_path)
    files, checks = build_bundle(catalog, suites, cfgs, witnesses)
    if any(c['status'] != 'PASS' for c in checks):raise ValueError(checks)
    episodes = [e for suite in suites for e in suite['episodes'] if e['split'] == 'dev']
    if {e['scenario_id'] for e in episodes} != {'S1','S2','S3','S4','S5'}:raise ValueError('missing dev scenario')
    selected = [e['episode_id'] for e in episodes]
    persist(files, destination/'certificates')
    report = dict(purpose='policy_dev_control_not_research', protocol='t1-0.2', platform=platform.platform(),
                  python=sys.version, certificates=checks, policies={})
    for policy in OFFICIAL_POLICIES:
        runs = ['run-1', args.b1_repeat if policy.startswith('b1-') else 'run-2']
        for run in runs:
            folder = args.output+'/'+policy+'/'+run
            if args.finalize_existing:
                if any(not (ROOT/folder/'attempts'/eid/'attempt-0001/result.json').is_file() for eid in selected):
                    raise ValueError('existing batch is incomplete: '+folder)
            else:
                print(json.dumps(dict(action='run_dev_controls',policy=policy,repeat=run,episodes=selected)),flush=True)
                run_batch(catalog_path, args.output+'/certificates', folder, policy, episode_ids=selected)
        controls = []
        replay = {}
        for episode in episodes:
            eid = episode['episode_id']
            a = destination/policy/runs[0]/'attempts'/eid/'attempt-0001'
            b = destination/policy/runs[1]/'attempts'/eid/'attempt-0001'
            result = json.loads((a/'result.json').read_bytes())
            runtime_error = result['end_reason'] in ('CRASH','PLANNER_TIMEOUT','WALL_TIMEOUT','INTERRUPTED','INVALID_STATE')
            expected = 'TIMEOUT' if episode['scenario_id'] == 'S5' else 'SUCCESS_REPORT'
            classification = 'runner_error_requires_investigation' if runtime_error else (
                'control_pass' if result['end_reason'] == expected and not (expected == 'TIMEOUT' and result['report'] is not None)
                else 'algorithm_or_implementation_failure_requires_trace_review')
            controls.append(dict(episode_id=eid,scenario=episode['scenario_id'],end_reason=result['end_reason'],
                classification=classification,goals_selected=result.get('goals_selected'),fallback_goals=result.get('fallback_goals'),
                unresolved_waypoints=result.get('unresolved_waypoints'),workers_reaped=result['workers_reaped']))
            replay[eid] = compare(a,b)
        # Resume must leave completed attempts and journal byte-exact.
        original = destination/policy/'run-1'
        protected = [original/'journal.jsonl',*[p for p in (original/'attempts').rglob('*') if p.is_file()]]
        before = {p:p.read_bytes() for p in protected}
        run_batch(catalog_path,args.output+'/certificates',args.output+'/'+policy+'/run-1',policy,episode_ids=selected,resume=True)
        unchanged = all(p.read_bytes() == data for p,data in before.items())
        report['policies'][policy] = dict(runs=runs,controls=controls,replay=replay,resume_files_unchanged=unchanged,protected_files=len(protected))
        print(json.dumps(dict(policy=policy,controls=controls,replay={eid:r['status'] for eid,r in replay.items()},resume_unchanged=unchanged)),flush=True)
    report['production_diff'] = subprocess.run(['git','diff','--name-only','HEAD','--','test version',
        ':!test version/benchmark/**',':!test version/test_benchmark_selectors.py'],cwd=ROOT,capture_output=True,text=True).stdout.splitlines()
    atomic(destination/'controls.json',encode(report))
    return 0 if all(all(c['classification'] == 'control_pass' for c in p['controls']) and all(r['status']=='PASS' for r in p['replay'].values())
                    and p['resume_files_unchanged'] for p in report['policies'].values()) and not report['production_diff'] else 1


if __name__ == '__main__':raise SystemExit(main())
