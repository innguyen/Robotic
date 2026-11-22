"""P1b.2 real batch/resume/kill/replay tests; no official baseline comparison."""
import copy
import csv
import json
import multiprocessing as mp
import os
from pathlib import Path
import threading
import time
import unittest
import shutil
from unittest.mock import patch
import test_benchmark_runner as single_tests
from benchmark.batch import run_batch
from benchmark.artifacts import encode,digest,output_path
from benchmark.schema import ROOT,load_json
from benchmark.journal import Journal,run_lock
from benchmark.replay import compare
from benchmark.lifecycle import process_alive


def killed_supervisor(catalog,bundle,output,connection,point,overrides):
    def checkpoint(kind,item):
        if kind==point:
            connection.send(item)
            threading.Event().wait()
    run_batch(catalog,bundle,output,episode_ids=['positive','empty'],policy_overrides=overrides,_hook=checkpoint)


class BatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        single_tests.RunnerTests.setUpClass()
        cls.base=single_tests.RunnerTests
        cls.count=0
        cls.kill_evidence=[]

    @classmethod
    def tearDownClass(cls):
        if os.environ.get('T1_BATCH_EXPORT'):
            target=output_path(os.environ['T1_BATCH_EXPORT'])
            (cls.base.directory/'kill-evidence.json').write_text(json.dumps(cls.kill_evidence,indent=2),encoding='utf8')
            shutil.copytree(cls.base.directory,target)
        single_tests.RunnerTests.tearDownClass()

    def folder(self):
        type(self).count+=1
        return self.base.relative+'/batch-'+str(type(self).count)

    def run_batch(self,out,**kwargs):return run_batch(self.base.catalog,self.base.bundle,out,**kwargs)

    def test_batch_multiple_primary_results_and_idempotent_resume(self):
        out=self.folder();s=self.run_batch(out,episode_ids=['positive','empty'])
        self.assertEqual(s['overall']['started'],2);self.assertEqual(s['attempts'],2)
        root=ROOT/out;before={str(p):p.read_bytes() for p in (root/'attempts').rglob('*') if p.is_file()}
        manifest=load_json(root/'run_manifest.json');self.assertEqual(manifest['protocol'],'t1-0.2');self.assertTrue(manifest['git_sha'])
        self.assertIn('positive',manifest['registry'])
        s2=self.run_batch(out,episode_ids=['positive','empty'],resume=True)
        self.assertEqual(s,s2)
        self.assertEqual(before,{str(p):p.read_bytes() for p in (root/'attempts').rglob('*') if p.is_file()})
        with (root/'episodes.csv').open(newline='') as f:
            rows=list(csv.DictReader(f));self.assertEqual(len(rows),2);self.assertIn('stalled_sim_s',rows[0])

    def test_worker_error_does_not_stop_next_episode(self):
        out=self.folder();s=self.run_batch(out,episode_ids=['positive','empty'],policy_overrides={'positive':'test-planner-hang'})
        self.assertEqual(s['overall']['execution_errors'],1);self.assertEqual(s['overall']['completed'],2)
        self.assertEqual(load_json(ROOT/out/'attempts/empty/attempt-0001/result.json')['end_reason'],'TIMEOUT')

    def test_retry_is_new_attempt_and_keeps_primary(self):
        out=self.folder();kwargs=dict(episode_ids=['positive'],policy='test-planner-error')
        self.run_batch(out,**kwargs)
        original=(ROOT/out/'attempts/positive/attempt-0001/result.json').read_bytes()
        s=self.run_batch(out,resume=True,retry_errors=True,**kwargs)
        self.assertEqual(s['attempts'],2);self.assertEqual(s['overall']['started'],1)
        self.assertEqual(original,(ROOT/out/'attempts/positive/attempt-0001/result.json').read_bytes())

    def test_successful_retry_never_replaces_failed_primary(self):
        from benchmark.batch import _execute_single as real
        out=self.folder()
        def transient(*args,**kwargs):
            return real(*args,fault='crash-after-report' if kwargs['attempt_id']==1 else None,**kwargs)
        with patch('benchmark.batch._execute_single',side_effect=transient):
            self.run_batch(out,episode_ids=['positive'],policy='test-report')
            s=self.run_batch(out,episode_ids=['positive'],policy='test-report',resume=True,retry_errors=True)
        self.assertEqual(load_json(ROOT/out/'attempts/positive/attempt-0002/result.json')['end_reason'],'SUCCESS_REPORT')
        self.assertEqual(s['overall']['mission_success'],0);self.assertEqual(s['overall']['execution_errors'],1)

    def test_interrupt_and_resume_before_first_execution(self):
        out=self.folder()
        def interrupt(kind,item):
            if kind=='reserved':raise KeyboardInterrupt()
        self.run_batch(out,episode_ids=['positive','empty'],_hook=interrupt)
        s=self.run_batch(out,episode_ids=['positive','empty'],resume=True)
        self.assertEqual(s['overall']['started'],2)
        self.assertEqual(load_json(ROOT/out/'attempts/positive/attempt-0001/result.json')['end_reason'],'INTERRUPTED')

    def kill_at(self,point,overrides):
        out=self.folder();ctx=mp.get_context('spawn');rx,tx=ctx.Pipe(False)
        proc=ctx.Process(target=killed_supervisor,args=(self.base.catalog,self.base.bundle,out,tx,point,overrides))
        proc.start();tx.close()
        try:
            self.assertTrue(rx.poll(30),'kill checkpoint not reached')
            item=rx.recv();folder=ROOT/out/item['path']
            pids=load_json(folder/'workers.json') if (folder/'workers.json').exists() else {}
            proc.kill();proc.join(5);self.assertFalse(proc.is_alive())
            deadline=time.monotonic()+5
            while any(process_alive(p) for p in pids.values()) and time.monotonic()<deadline:time.sleep(.05)
            self.assertTrue(all(not process_alive(p) for p in pids.values()),'orphan worker after supervisor kill')
            type(self).kill_evidence.append(dict(checkpoint=point,output=out,supervisor_pid=proc.pid,supervisor_exitcode=proc.exitcode,
                                                 workers=pids,workers_alive_after_kill={name:process_alive(pid) for name,pid in pids.items()}))
        finally:
            if proc.is_alive():proc.terminate();proc.join(5)
            rx.close()
        return out,folder

    def test_real_supervisor_kill_lifeline_and_resume(self):
        overrides={'positive':'test-policy-hang'}
        out,folder=self.kill_at('running',overrides)
        s=self.run_batch(out,episode_ids=['positive','empty'],policy_overrides=overrides,resume=True)
        self.assertEqual(s['overall']['completed'],2)
        self.assertEqual(load_json(folder/'result.json')['end_reason'],'CRASH')
        self.assertEqual(s['attempts'],2)

    def test_kill_after_atomic_result_adopts_without_overwrite(self):
        out,folder=self.kill_at('result',{})
        before=(folder/'result.json').read_bytes()
        self.run_batch(out,episode_ids=['positive','empty'],resume=True)
        self.assertEqual(before,(folder/'result.json').read_bytes())

    def test_report_matching_survives_recovery_without_result(self):
        out,folder=self.kill_at('result',{})
        # Simulate a result write lost before commit; source event log stays intact.
        (folder/'result.json').unlink()
        self.run_batch(out,episode_ids=['positive','empty'],resume=True)
        r=load_json(folder/'result.json')
        self.assertTrue(r['report_correct']);self.assertFalse(r['mission_success']);self.assertEqual(r['end_reason'],'CRASH')

    def test_resume_rejects_policy_or_completed_file_changes(self):
        out=self.folder();self.run_batch(out,episode_ids=['positive'])
        with self.assertRaises(ValueError):self.run_batch(out,episode_ids=['positive'],policy='test-idle',resume=True)
        result=ROOT/out/'attempts/positive/attempt-0001/result.json'
        original=result.read_bytes();result.write_bytes(original+b' ')
        with self.assertRaises(ValueError):self.run_batch(out,episode_ids=['positive'],resume=True)
        result.write_bytes(original)

    def test_journal_truncated_tail_and_corruption(self):
        path=self.base.directory/'journal-test.jsonl';j=Journal(path);j.append('ONE',value=1)
        with path.open('ab') as f:f.write(b'{"seq":2')
        repaired=Journal(path);self.assertEqual(len(repaired.records),1);repaired.append('TWO',value=2)
        self.assertEqual(len(Journal(path).records),2)
        raw=path.read_bytes().replace(b'"value":1',b'"value":9');path.write_bytes(raw)
        with self.assertRaises(ValueError):Journal(path)

    def test_os_lock_prevents_concurrent_supervisors(self):
        folder=self.base.directory/'lock-test';folder.mkdir()
        with run_lock(folder):
            with self.assertRaises(ValueError):
                with run_lock(folder):pass

    def test_atomic_replace_retries_transient_lock(self):
        from benchmark.journal import atomic
        real=os.replace;calls=[];target=self.base.directory/'locked-projection.json'
        def transient(a,b):
            calls.append(1)
            if len(calls)==1:raise PermissionError('injected transient lock')
            return real(a,b)
        with patch('benchmark.journal.os.replace',side_effect=transient):atomic(target,b'{"ok":true}')
        self.assertEqual(load_json(target),{'ok':True});self.assertEqual(len(calls),2)

    def test_full_corpus_validated_before_filter_and_resume(self):
        original=self.base.directory/'offset.json';data=original.read_bytes();bad=json.loads(data);bad['episodes'][0]['layout_hash']='0'*64
        original.write_text(json.dumps(bad))
        try:
            with self.assertRaises(ValueError):self.run_batch(self.folder(),episode_ids=['positive'])
        finally:original.write_bytes(data)

    def test_cli_malformed_yaml_is_input_error_before_output(self):
        from benchmark.batch import main
        original=self.base.directory/'base.json';before=original.read_bytes();suite=json.loads(before)
        malformed=self.base.directory/'malformed.yaml';malformed.write_text('[unclosed')
        suite['shared']['navigation_yaml']=malformed.relative_to(ROOT).as_posix();original.write_text(json.dumps(suite))
        out=self.folder()
        try:
            args=['benchmark_batch.py','--catalog',self.base.catalog,'--bundle',self.base.bundle,'--output',out]
            with patch('sys.argv',args):self.assertEqual(main(),2)
            self.assertFalse((ROOT/out).exists())
        finally:original.write_bytes(before)

    def test_strict_replay_and_deliberate_divergence(self):
        a,b=self.folder(),self.folder()
        for out in (a,b):self.run_batch(out,episode_ids=['positive'])
        left,right=(ROOT/p/'attempts/positive/attempt-0001' for p in (a,b))
        self.assertEqual(compare(left,right)['status'],'PASS')
        trajectory=right/'trajectory.csv';trajectory.write_text(trajectory.read_text().replace('0.0','0.001',1))
        self.assertEqual(compare(left,right)['status'],'FAIL')

    def test_vectorized_coverage_matches_scalar_definition(self):
        import math
        import numpy as np
        from navigation_core_ros1 import Grid
        from benchmark.coverage import coverage_cells,camera_pose
        rng=np.random.default_rng(4)
        for yaw in (0.,.3,-2.,math.pi):
            raw=rng.choice([0,0,0,-1,1],size=(40,40));raw[15:25,15:25]=0
            grid=Grid(raw,np.ones(raw.shape,dtype=bool),[-20,-20],.1)
            camera=dict(x_m=.15,y_m=-.08,yaw_rad=.2,fov_deg=72.,range_m=1.5)
            pose=(0.,0.,yaw);x,y,heading=camera_pose(pose,camera);half=math.radians(72.)/2
            rays=max(2,math.ceil(2*half/max(.005,.1/(2*1.5))));expected=set();step=.05
            for i in range(rays+1):
                angle=heading-half+2*half*i/rays
                for j in range(math.floor(1.5/step)+1):
                    px,py=x+math.cos(angle)*j*step,y+math.sin(angle)*j*step
                    if not grid.inside((px,py)) or grid.raw[grid.cell((px,py))]!=0:break
                    expected.add((math.floor(px/.1),math.floor(py/.1)))
            self.assertEqual(coverage_cells(grid,pose,camera),frozenset(expected))

    def test_zero_denominators_and_mission_timeout_not_retried(self):
        out=self.folder();s=self.run_batch(out,episode_ids=['empty'])
        self.assertIsNone(s['overall']['ReportSuccess']);self.assertIsNone(s['scenarios']['S1']['negative_false_alarm_rate'])
        self.assertEqual(self.run_batch(out,episode_ids=['empty'],resume=True,retry_errors=True)['attempts'],1)


if __name__=='__main__':unittest.main()
