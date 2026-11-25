"""Entry point: actual SearchMission/Navigator, never the S0 smoke policy."""
import json
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from controller import Supervisor
from runtime import JsonLog, default_config, run_core


if __name__ == "__main__":
    sim = Supervisor()
    config = default_config()
    if len(sys.argv) > 1:
        config.update(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")))
    try:
        run_core(sim, config)
    except Exception:
        JsonLog(config.get("log")).emit(event="CORE_ERROR", traceback=traceback.format_exc())
        sim.getSelf().setVelocity([0.]*6)
        sim.simulationQuit(1)
        sim.step(32)
        raise
