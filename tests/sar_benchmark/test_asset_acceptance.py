"""Asset authoring/privacy and independent acceptance regression fixtures."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from sar_benchmark.artifacts import RunStore,write_json
from sar_benchmark.audit_run import audit
from sar_benchmark.contracts import ContractError,canonical,public_copy
from sar_benchmark.episode import Episode
from sar_benchmark.evaluator import evaluate
from sar_benchmark.scenario_contract import validate_scenario
from sar_benchmark.scenarios import sar_scenario
from sar_benchmark.admissibility import reachable_grid
from test_pipeline import FixtureBackend,observation


class AssetTests(unittest.TestCase):
    def test_reference_surfaces_are_bound_to_the_native_layout(self):
        for environment in ("indoor","outdoor"):
            case = sar_scenario(environment=environment,scene_set="reference")
            validate_scenario(case)
            historical = sar_scenario(environment=environment,scene_set="assets")
            self.assertEqual(case["victims"],historical["victims"])
            self.assertNotEqual(case["family"],historical["family"])
            self.assertTrue(any(x>4 for x,y in reachable_grid(case)))
            proxy = next(b for b in case["boxes"] if b.get("mesh")=="reference_surface")
            proxy["center_m"][0] += 1.
            with self.assertRaises(ContractError):
                validate_scenario(case)

    def test_site_obj_import_places_asymmetric_geometry_in_authored_robot_frame(self):
        # Native UE OBJ import reflects Y once. An off-centre triangle makes
        # a second reflection and reversed winding independently observable.
        from experiments.ue5_s4.bake_environment import Mesh
        mesh = Mesh()
        triangle = [(2,3,1),(3,3,1),(2,4,1)]
        mesh.triangle(triangle)
        with tempfile.TemporaryDirectory() as folder:
            obj = Path(folder)/"site.obj"
            mesh.save(obj,site_coordinates=True)
            lines = obj.read_text().splitlines()
        imported = [(float(x)/100,-float(y)/100,float(z)/100)
                    for x,y,z in (line.split()[1:] for line in lines if line.startswith("v "))]
        self.assertEqual(imported,[(2,-3,1),(3,-3,1),(2,-4,1)])
        self.assertIn("f 1/1 2/2 3/3",lines)

    def test_layouts_are_distinct_reachable_and_private(self):
        configs=[sar_scenario(scene_set="assets",environment=e) for e in ("indoor","outdoor")]
        self.assertNotEqual(configs[0]["boxes"],configs[1]["boxes"])
        self.assertNotEqual(configs[0]["victims"][0]["anchor_m"][:2],configs[1]["victims"][0]["anchor_m"][:2])
        for config in configs:
            validate_scenario(config)
            self.assertTrue(any(x>4 for x,y in reachable_grid(config)))
            for key in ("scene_variant","family","boxes"):
                obs=observation();obs[key]=config[key]
                with self.assertRaises(ContractError):public_copy(obs)
            bad=copy.deepcopy(config);bad["boxes"][0]["mesh"]="../../private-canary"
            with self.assertRaises(ContractError):validate_scenario(bad)
        bad=copy.deepcopy(configs[0]);bad["scene_variant"]="outdoor_damaged_assets_v1"
        with self.assertRaises(ContractError):validate_scenario(bad)

    def test_anatomy_torso_origins_floor_and_locked_bytes(self):
        root=Path(__file__).resolve().parents[2]/"experiments/ue5_s4/assets/anatomy"
        poses=json.loads((root/"dressed/poses.json").read_text())
        self.assertEqual(set(poses),{"standing","seated","prone"})
        for posture,item in poses.items():
            self.assertEqual(hashlib.sha256((root/"dressed"/item["file"]).read_bytes()).hexdigest(),item["sha256"])
            self.assertEqual(set(item["material_slots"]),{"skin","eyes","shirt","trousers","footwear","bob"})
            self.assertAlmostEqual(item["bounds_relative_m"][0][2]+item["torso_height_m"],0.)
            config=sar_scenario(posture=posture,scene_set="assets")
            self.assertEqual(config["victims"][0]["anchor_m"][2],item["torso_height_m"])
        self.assertGreater(poses["standing"]["torso_height_m"],poses["seated"]["torso_height_m"])
        self.assertGreater(poses["seated"]["torso_height_m"],poses["prone"]["torso_height_m"])


class AcceptanceTests(unittest.TestCase):
    def test_interrupted_attempt_is_rejected_without_repairing_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/"private").mkdir()
            for index in range(54):(root/"private"/f"e{index:03d}-ledger.json").write_text("{}")
            result=audit(root)
            self.assertEqual(result["status"],"FAIL")
            self.assertEqual(result["reference_episodes"],0)
            self.assertFalse((root/"inventory.json").exists())

    def make_run(self,path,mutation=None,calibration=None,omit_public=False):
        # Synthetic contract fixture only; never presented as engine evidence.
        store=RunStore(path)
        episode=Episode(FixtureBackend(),{},3);obs=episode.reset()
        episode.submit({"epoch":1,"sequence":0,"tick":0,"twist":[0,0,0],"reports":[],"finish":True})
        data=episode.ledger
        data["frames"]["0"]["delivered_rgb_sha256"]=hashlib.sha256(canonical(obs["rgb"]["data"])).hexdigest()
        metrics=evaluate(data)
        manifest={"source_sha256":{},"rgbd_size":[8,8],"methods":["sweep"],"calibrate":False}
        result={"status":"COMPLETED_DIAGNOSTIC_UE_RUNTIME","engine_exit_code":0,"rows":[{"episode":"e000","metrics":copy.deepcopy(metrics)}]}
        scheduled=[{}]
        if mutation:mutation(manifest,result,scheduled,metrics)
        if calibration is not None:
            manifest["calibrate"]=True
            write_json(path/"calibration.json",calibration)
        for name,value in (("manifest.json",manifest),("host-capabilities.json",{}),("results.json",result),
            ("private/e000-ledger.json",data),("private/e000-worker.json",{"exit_code":0}),
            ("e000-metrics.json",{"metrics":metrics}),("private/scheduled-scenarios.json",scheduled)):
            write_json(path/name,value)
        with gzip.open(path/"public/observations.jsonl.gz","wt") as stream:
            if not omit_public:
                stream.write(json.dumps({"observation":obs})+"\n")
        with gzip.open(path/"private/host.jsonl.gz","wt") as stream:
            stream.write(json.dumps({"capture_start_wall_ns":1,"capture_end_wall_ns":2,"available_wall_ns":3})+"\n")
        store.event({"type":"engine_exit","exit_code":0});store.close()
        return audit(path)

    def test_sealed_empty_public_stream_cannot_validate_a_finished_episode(self):
        with tempfile.TemporaryDirectory() as folder:
            result=self.make_run(Path(folder)/"run",omit_public=True)
        self.assertEqual(result["status"],"FAIL")
        self.assertIn("e000-ledger.json/0: missing delivered observation",result["errors"])

    def test_schedule_count_alone_cannot_validate_method_and_scene_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            result=self.make_run(Path(folder)/"run",lambda manifest,*_:manifest.update(runner_arguments={}))
        self.assertEqual(result["scheduled_episode_count"],1)
        self.assertEqual(result["verified_scheduled_episode_identities"],0)
        self.assertIn("e000: method or private scene differs from schedule",result["run_acceptance_errors"])

    def test_completion_calibration_and_metrics_cannot_be_inferred_from_count(self):
        cases=[(None,True),
            (lambda m,r,s,t:r.update(status="FAILED_CALIBRATION"),False),
            (lambda m,r,s,t:s.extend([{}]*53),False),
            (lambda m,r,s,t:m.update(calibrate=True),False),
            (lambda m,r,s,t:r["rows"][0]["metrics"].update(tp=99),False),
            (lambda m,r,s,t:t.update(tp=99),False)]
        with tempfile.TemporaryDirectory() as directory:
            for index,(mutation,accepted) in enumerate(cases):
                with self.subTest(index=index):
                    result=self.make_run(Path(directory)/str(index),mutation)
                    self.assertEqual(result["status"]=="PASS_OFFLINE_AUDIT",accepted)

    def test_calibration_pass_requires_observed_checks_and_referenced_probe(self):
        with tempfile.TemporaryDirectory() as directory:
            for index, flags in enumerate(({}, {"reset_render_repeat": False},
                                          {"asset_native_geometry_probe": True})):
                with self.subTest(flags=flags):
                    result = self.make_run(Path(directory)/str(index), calibration={
                        "status": "PASS_ENGINE_CALIBRATION", "checks": flags})
                    self.assertEqual(result["status"], "FAIL")
                    self.assertEqual(result["requested_calibration_status"], "FAIL_INCONSISTENT_CALIBRATION")


if __name__=="__main__":unittest.main()
