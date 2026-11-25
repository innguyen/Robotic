"""Trusted lifecycle and blind report acknowledgement; no scoring in this module."""
from enum import Enum

from .contracts import ContractError, DT_NS, MAX_REPORTS, canonical, public_copy, validate_action
from .safety import SafetyGate


class Phase(Enum):
    NEW = "NEW"
    RUNNING = "RUNNING"
    TERMINAL = "TERMINAL"
    CLOSED = "CLOSED"


class Episode:
    def __init__(self, backend, scenario, budget_ticks=30):
        self.backend, self.scenario = backend, scenario
        self.budget_ticks = budget_ticks
        self.phase = Phase.NEW
        self.safety = SafetyGate()
        self.sequence = 0
        self.obs = None
        self.ledger = None
        self.commands = []
        self.violations = []

    def reset(self):
        if self.phase not in (Phase.NEW, Phase.TERMINAL):
            raise ContractError("reset while active")
        self.backend.stop()
        # Backend reset must succeed completely before exposing a new epoch.
        obs, truth = self.backend.reset(self.scenario, self.budget_ticks)
        public_copy(obs)
        if self.ledger is not None and obs["epoch"] <= self.ledger["epoch"]:
            raise ContractError("reset did not advance epoch")
        self.sequence = 0
        self.safety = SafetyGate()
        self.commands, self.violations = [], []
        self.ledger = {"epoch": obs["epoch"], "budget_ns": self.budget_ticks * DT_NS,
                       "victims": truth["victims"], "frames": {}, "reports": [],
                       "safety_events": [], "path_m": 0.0, "terminal_reason": None, "terminal_ns": None}
        self.phase, self.obs = Phase.RUNNING, obs
        self._record(truth)
        return public_copy(obs)

    def _record(self, truth):
        self.ledger["frames"][str(self.obs["sequence"])] = truth["frame"]
        self.ledger["safety_events"] = list(truth["safety_events"])
        self.ledger["path_m"] = truth["path_m"]

    def submit(self, action):
        if self.phase != Phase.RUNNING:
            raise ContractError("episode not running")
        try:
            validate_action(action, self.obs["epoch"], self.sequence, self.obs["tick"])
        except ContractError:
            self.terminate("AGENT_PROTOCOL_ERROR")
            raise
        acks = []
        for payload in action["reports"]:
            if len(self.ledger["reports"]) >= MAX_REPORTS:
                self.violations.append("REPORT_LIMIT")
                acks.append({"received": False, "reason": "report_limit"})
                continue
            # Freeze payload now: caller mutation after ACK cannot edit reports.
            from .contracts import strict_loads
            self.ledger["reports"].append({"payload": strict_loads(canonical(payload)),
                                            "accepted_ns": self.obs["sim_time_ns"]})
            acks.append({"received": True, "slot": len(self.ledger["reports"] )})
        if action["finish"]:
            self.terminate("FINISH")
            return None, acks
        checked, reason = self.safety.check(action["twist"], self.obs)
        entry = {"epoch": self.obs["epoch"], "tick": self.obs["tick"], "desired": action["twist"],
                 "limited": self.safety.last_limited, "checked": checked, "safety_reason": reason}
        self.commands.append(entry)
        try:
            obs, truth = self.backend.advance(checked)
            public_copy(obs)
            if obs["epoch"] != self.obs["epoch"] or obs["tick"] != self.obs["tick"] + 1 or obs["sequence"] != self.obs["sequence"] + 1:
                raise ContractError("backend time/epoch discontinuity")
            entry["applied"] = truth["applied"]
            self.obs = obs
            self._record(truth)
        except Exception:
            self.terminate("INFRA_ERROR")
            raise
        self.sequence += 1
        if self.ledger["safety_events"]:
            self.terminate("SAFETY_ABORT")
        elif self.obs["tick"] >= self.budget_ticks:
            self.terminate("TIMEOUT")
        return public_copy(self.obs) if self.phase == Phase.RUNNING else None, acks

    def terminate(self, reason):
        if self.phase == Phase.RUNNING:
            self.backend.stop()
            self.ledger["terminal_reason"] = reason
            self.ledger["terminal_ns"] = self.obs["sim_time_ns"]
            self.phase = Phase.TERMINAL

    def close(self):
        if self.phase == Phase.RUNNING:
            self.terminate("AGENT_ERROR")
        self.backend.close()
        self.phase = Phase.CLOSED
