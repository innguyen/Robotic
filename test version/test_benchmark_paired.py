"""Actual spawned paired jobs, locked identities, failures/retries and raw reduction."""
import copy
import csv
import json
import os
import shutil
import unittest
from unittest.mock import patch

import test_benchmark_runner as single_tests
from benchmark.artifacts import encode,digest,load_catalog,build_bundle,persist,output_path
from benchmark.schema import ROOT,load_json
from benchmark.selectors import B0,B1
from benchmark.paired import initialize,job,audit,load_pair,read_job,journal_records,paired_rows
from benchmark.batch import run_batch as actual_batch
from benchmark.journal import run_lock


def fail_bootstrap(connection,public,policy_id,lifeline=None):
    connection.send({'ready':False});connection.close()


class PairedIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        single_tests.RunnerTests.setUpClass();cls.base=single_tests.RunnerTests;cls.counter=0;cls.cases=[]

    @classmethod
    def tearDownClass(cls):
        if os.environ.get('T1_P22_TEST_EXPORT'):
            destination=output_path(os.environ['T1_P22_TEST_EXPORT'])
            (cls.base.directory/'p22-cases.json').write_bytes(encode(cls.cases))
            shutil.copytree(cls.base.directory,destination)
        single_tests.RunnerTests.tearDownClass()

    def pair(self,selection=('positive','empty')):
        type(self).counter+=1;output=self.base.relative+'/paired-'+str(self.counter)
        initialize(self.base.catalog,self.base.bundle,output,list(selection))
        type(self).cases.append(dict(test=self._testMethodName,archive='paired-'+str(self.counter)))
        return output

    def test_real_pair_metrics_replay_csv_and_idempotent_audit(self):
        output=self.pair()
        for replica in ('primary','replay'):
            for policy in (B0,B1):
                for eid in ('positive','empty'):job(output,policy,replica,eid)
        report=audit(output);self.assertEqual(report['status'],'PASS',report['issues'])
        for summary in report['summaries'].values():
            self.assertEqual(summary['started'],2);self.assertEqual(summary['ReportSuccess'],1.)
            self.assertEqual(summary['SafeMissionSuccess'],1.);self.assertEqual(summary['negative_false_alarm_rate'],0.)
        with (ROOT/output/'analysis/paired-results.csv').open(newline='') as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),2)
        self.assertEqual(rows[0]['b0_end_reason'],'SUCCESS_REPORT');self.assertEqual(rows[1]['b1_end_reason'],'TIMEOUT')
        self.assertAlmostEqual(float(rows[0]['b1_elapsed_sim_s']),.4)
        self.assertTrue(all(r['b0_attempt_id']=='1' and r['b1_attempt_id']=='1' for r in rows))
        with (ROOT/output/'analysis/attempts.csv').open(newline='') as stream:attempt=next(csv.DictReader(stream))
        self.assertGreater(json.loads(attempt['monitor_state_seconds'])['IDLE'],0.)
        with (ROOT/output/'analysis/all-attempts.csv').open(newline='') as stream:all_rows=list(csv.DictReader(stream))
        self.assertEqual(len(all_rows),8);self.assertEqual({r['replica'] for r in all_rows},{'primary','replay'})
        protected={p:p.read_bytes() for p in (ROOT/output/'runs').rglob('*') if p.is_file()}
        second=audit(output);self.assertEqual(report,second)
        self.assertEqual(protected,{p:p.read_bytes() for p in (ROOT/output/'runs').rglob('*') if p.is_file()})

    def test_rehashed_identity_tampering_is_rejected(self):
        output=self.pair();folder=ROOT/output;original=load_json(folder/'paired_manifest.json')
        variants=[]
        for key in ('shared_hash','config_hash','resolved_bytes_hash','budget_s'):
            v=copy.deepcopy(original);v['fixtures']['positive'][key]='tampered';variants.append(v)
        v=copy.deepcopy(original);v['metrics']['version']='different';variants.append(v)
        v=copy.deepcopy(original);v['evaluator_sources']['evaluator.py']='bad';variants.append(v)
        v=copy.deepcopy(original);v['policies'][B0]['positive']['selector_parameters']['spacing_m']=.5;variants.append(v)
        v=copy.deepcopy(original);v['selection'].append('positive');variants.append(v)
        v=copy.deepcopy(original);v['execution_profile']['max_active_jobs']=2;variants.append(v)
        for v in variants:
            raw=encode(v);(folder/'paired_manifest.json').write_bytes(raw);(folder/'paired_manifest.sha256').write_text(digest(raw)+'\n')
            with self.assertRaises(ValueError):load_pair(output)
        raw=encode(original);(folder/'paired_manifest.json').write_bytes(raw);(folder/'paired_manifest.sha256').write_text(digest(raw)+'\n')

    def test_missing_pair_and_incomplete_journal_never_pass_or_repair(self):
        output=self.pair(('positive',))
        with self.assertRaises(OSError):audit(output)
        self.assertFalse((ROOT/output/'analysis/audit.json').exists())
        job(output,B0,'primary','positive')
        spec=load_pair(output);folder=ROOT/spec['slots']['primary'][B0]['positive']
        path=folder/'journal.jsonl';original=path.read_bytes();path.write_bytes(original[:-1])
        with self.assertRaises(ValueError):journal_records(path)
        self.assertEqual(path.read_bytes(),original[:-1])
        path.write_bytes(original)

    def test_batch_and_completed_attempt_tampering_rejected(self):
        output=self.pair(('positive',));job(output,B0,'primary','positive');spec=load_pair(output)
        folder=ROOT/spec['slots']['primary'][B0]['positive'];path=folder/'run_manifest.json';original=path.read_bytes()
        data=json.loads(original);data['identity']['runtime']['platform']='foreign';path.write_bytes(encode(data))
        with self.assertRaisesRegex(ValueError,'identity'):read_job(spec,B0,'primary','positive')
        path.write_bytes(original)
        result=folder/'attempts/positive/attempt-0001/result.json';original=result.read_bytes();data=json.loads(original)
        data['mission_success']=False;result.write_bytes(encode(data))
        with self.assertRaisesRegex(ValueError,'modified'):read_job(spec,B0,'primary','positive')
        result.write_bytes(original)

    def test_execution_error_and_successful_retry_remain_in_primary_pair(self):
        output=self.pair(('positive',))
        for replica in ('primary','replay'):
            with patch('benchmark.runner.policy_worker',fail_bootstrap):job(output,B0,replica,'positive')
            job(output,B0,replica,'positive',resume=True,retry_errors=True)
            job(output,B1,replica,'positive')
        report=audit(output);self.assertEqual(report['status'],'PASS',report['issues'])
        summary=report['summaries'][B0]
        self.assertEqual(summary['attempts'],2);self.assertEqual(summary['execution_errors'],1)
        self.assertEqual(summary['ReportSuccess'],0.);self.assertEqual(summary['SafeMissionSuccess'],0.)
        with (ROOT/output/'analysis/paired-results.csv').open(newline='') as stream:row=next(csv.DictReader(stream))
        self.assertEqual(row['b0_end_reason'],'CRASH');self.assertEqual(row['b0_mission_success'],'False')
        with (ROOT/output/'analysis/attempts.csv').open(newline='') as stream:rows=list(csv.DictReader(stream))
        self.assertTrue(any(r['policy_id']==B0 and r['attempt_id']=='2' and r['mission_success']=='True' for r in rows))

    def test_layout_hash_alone_cannot_pair_different_fixture(self):
        output=self.pair(('positive',));spec=load_pair(output)
        # A same-layout row with a different full fixture identity must not pair.
        row={**spec['fixtures']['positive']['environment'], 'policy_id':B0,'episode_id':'positive','attempt_id':1,'fixture_hash':'wrong'}
        with self.assertRaises(ValueError):paired_rows(spec,[dict(metrics=row)])

    def test_test_split_selection_rejected_after_full_corpus_validation(self):
        name='test version/benchmark/fixtures/catalog.json';catalog,suites,cfgs,witnesses=load_catalog(name)
        files,results=build_bundle(catalog,suites,cfgs,witnesses)
        self.assertTrue(all(r['status']=='PASS' for r in results))
        target=self.base.directory/'static-bundle';persist(files,target)
        with self.assertRaisesRegex(ValueError,'cannot select test'):
            initialize(name,target.relative_to(ROOT).as_posix(),self.base.relative+'/invalid-test-pair',['test-empty-01'])

    def test_pair_global_job_lock_rejects_concurrent_admission(self):
        output=self.pair(('positive',))
        with run_lock(ROOT/output):
            with self.assertRaisesRegex(ValueError,'live supervisor'):
                job(output,B0,'primary','positive')
        self.assertFalse((ROOT/output/'runs').exists())
        self.assertFalse((ROOT/output/'bindings').exists())

    def test_reserved_before_manifest_recovery_keeps_unknowns_and_primary(self):
        output=self.pair(('positive',))
        def interrupted(*args,**kwargs):
            def checkpoint(kind,item):
                if kind=='reserved':raise KeyboardInterrupt()
            return actual_batch(*args,**kwargs,_hook=checkpoint)
        with patch('benchmark.paired.run_batch',side_effect=interrupted):job(output,B0,'primary','positive')
        job(output,B0,'primary','positive',resume=True)
        spec=load_pair(output);rows=read_job(spec,B0,'primary','positive')
        self.assertEqual(rows[0]['metrics']['end_reason'],'INTERRUPTED');self.assertEqual(rows[0]['measurement_status'],'UNVERIFIED')
        self.assertEqual(rows[0]['identity_basis'],'batch_reservation_no_execution_evidence')
        self.assertIsNone(rows[0]['metrics']['path_length_m']);self.assertIsNone(rows[0]['metrics']['goals_selected'])
        self.assertFalse((ROOT/rows[0]['attempt_path']/'run_manifest.json').exists())
        job(output,B0,'primary','positive',resume=True,retry_errors=True)
        rows=read_job(spec,B0,'primary','positive')
        self.assertEqual(len(rows),2);self.assertFalse(rows[0]['metrics']['mission_success']);self.assertTrue(rows[1]['metrics']['mission_success'])


if __name__=='__main__':unittest.main()
