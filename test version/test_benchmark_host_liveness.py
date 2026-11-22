"""Host diagnostic vectors and spawned compatibility without scoring changes."""
import json
import os
from pathlib import Path
import tempfile
import unittest

import test_benchmark_runner as single
from benchmark.artifacts import encode,output_path
from benchmark.host_liveness import review,clocks,process_cpu,tail,Recorder
from benchmark import stabilization,paired
from benchmark.selectors import B0
from benchmark.replay import compare
from benchmark.journal import run_lock


def sample(seq,mono,active,*,boot=None,utc=None,step=0,cpu=0.):
    return dict(seq=seq,clock=dict(monotonic_s=mono,utc_s=mono if utc is None else utc,
        boot_s=mono if boot is None else boot,active_s=active),
        progress=dict(step_index=step),worker_cpu_s={'policy_pid':cpu})


class HostDiagnosticTests(unittest.TestCase):
    def test_continuous_host_and_simulated_progress(self):
        report=review([sample(1,0.,0.),sample(2,.5,.5,step=1,cpu=.1)])
        self.assertEqual(report['status'],'PASS');self.assertEqual(report['suspend_gaps'],[])

    def test_suspend_advances_biased_clock_but_not_active(self):
        report=review([sample(1,0.,0.),sample(2,40.,.5)])
        self.assertEqual(report['status'],'UNVERIFIED');self.assertAlmostEqual(report['suspend_gaps'][0]['unbiased_gap_s'],39.5)

    def test_observer_gap_without_suspend_is_not_called_suspend(self):
        report=review([sample(1,0.,0.),sample(2,8.,8.)])
        self.assertEqual(report['status'],'UNVERIFIED');self.assertEqual(report['suspend_gaps'],[])
        self.assertIn('host observer sampling gap',report['issues'])

    def test_cpu_busy_vs_quiet_are_observations_not_causal_claims(self):
        r=review([sample(1,0.,0.),sample(2,.5,.5,cpu=.2),sample(3,1.,1.,cpu=.2)])
        self.assertEqual([x['observation'] for x in r['no_tick_observations']],['CPU_ACTIVE_NO_TICK','CPU_QUIET_OR_UNAVAILABLE_NO_TICK'])
        self.assertIn('UNVERIFIED',r['planner_heartbeat']);self.assertIn('cannot distinguish',r['causal_limit'])

    def test_wall_clock_adjustment_does_not_extend_watchdog_or_imply_suspend(self):
        r=review([sample(1,0.,0.),sample(2,.5,.5,utc=100.)])
        self.assertEqual(r['status'],'PASS');self.assertEqual(r['utc_jump_sequences'],[2]);self.assertEqual(r['suspend_gaps'],[])

    def test_missing_clocks_errors_and_bad_sequences_never_pass(self):
        for rows in ([],[sample(1,0.,0.)],[sample(1,0.,None),sample(2,.5,None)],
                     [sample(1,1.,1.),sample(3,0.,0.)]):
            self.assertEqual(review(rows)['status'],'UNVERIFIED')
        rows=[sample(1,0.,0.),sample(2,.5,.5)];rows[-1]['error']='OSError'
        self.assertEqual(review(rows)['status'],'UNVERIFIED')

    def test_actual_clock_and_cpu_read_without_power_mutation(self):
        a=clocks();self.assertGreater(a['monotonic_s'],0.);self.assertGreaterEqual(process_cpu(os.getpid()),0.)
        self.assertIsNone(process_cpu(0x7fffffff))

    def test_tail_ignores_torn_record_and_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'tail';path.write_bytes(b'old\n'+b'x'*10000+b'\ncomplete\ntorn')
            self.assertEqual(tail(path),'complete');self.assertIsNone(tail(Path(directory)/'missing'))

    def test_observer_thread_failure_is_not_a_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            with Recorder(Path(directory)/'host',Path(directory)/'slot') as recorder:
                recorder.thread_error='OSError'
            self.assertEqual(recorder.summary['status'],'UNVERIFIED')


class HostPairedIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        single.RunnerTests.setUpClass();cls.base=single.RunnerTests

    @classmethod
    def tearDownClass(cls):single.RunnerTests.tearDownClass()

    def test_real_sidecar_semantics_match_uninstrumented_and_tamper_rejected(self):
        base=self.base;output=base.relative+'/stabilized'
        paired.initialize(base.catalog,base.bundle,output,['positive']);stabilization.initialize(output)
        result=stabilization.job(output,B0,'primary','positive');self.assertEqual(result['status'],'PASS')
        directory=output_path(output);spec=paired.load_pair(output)
        raw=spec['slots']['primary'][B0]['positive']+'/attempts/positive/attempt-0001'
        control=base.relative+'/control'
        single.run_single(base.catalog,base.bundle,'positive',control,B0)
        self.assertEqual(compare(output_path(raw),output_path(control))['status'],'PASS')
        host=directory/'host/primary'/B0/'positive';report=json.loads((host/'result.json').read_bytes())
        self.assertEqual(report['diagnostics']['status'],'PASS')
        self.assertIsNotNone(json.loads((host/'samples.jsonl').read_bytes().splitlines()[-1])['progress']['sim_time_s'])
        protected=(host/'samples.jsonl').read_bytes()
        with self.assertRaisesRegex(ValueError,'existing slot'):stabilization.job(output,B0,'primary','positive')
        self.assertEqual((host/'samples.jsonl').read_bytes(),protected)
        with self.assertRaisesRegex(ValueError,'fresh paired'):stabilization.initialize(output)
        with (host/'samples.jsonl').open('ab') as stream:stream.write(encode(sample(999,1.,1.))+b'\n')
        with self.assertRaisesRegex(ValueError,'host evidence'):stabilization.audit(output)

    def test_admission_and_rehashed_host_profile_change_fail_closed(self):
        base=self.base;output=base.relative+'/stabilization-lock'
        paired.initialize(base.catalog,base.bundle,output,['positive']);stabilization.initialize(output)
        with run_lock(output_path(output)):
            with self.assertRaisesRegex(ValueError,'live supervisor'):stabilization.job(output,B0,'primary','positive')
        path=output_path(output)/'stabilization_manifest.json';value=json.loads(path.read_bytes());value['observer_gap_s']=100.
        path.write_bytes(encode(value))
        with self.assertRaisesRegex(ValueError,'identity changed'):stabilization.job(output,B0,'primary','positive')
        self.assertFalse((output_path(output)/'host').exists())


if __name__=='__main__':unittest.main()
