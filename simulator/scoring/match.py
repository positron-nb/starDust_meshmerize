"""Full match simulation: calibration minute, dry run, hand-over, actual run,
restarts, official clocks and the referee's view (checkpoints, LED, end box,
shortest-path judgement).

Referee model (assumptions, configurable in MatchConfig):
  * A checkpoint is "crossed" when the robot's axle centre passes within
    checkpoint_radius_mm of the checkpoint reference point (the rulebook only
    says the circles are references; ask the organisers - see docs).
  * Dry-run restart: the robot is placed at the LAST checkpoint it crossed
    (START if none), facing the way it left that checkpoint.
  * Actual-run restart: the robot is placed at START.
  * Clocks are never paused; restart handling time is added to the run clock.
  * The actual run counts as "shortest path" when the ROUTE of the final
    attempt is a shortest START->box route (any equally short route
    qualifies), as a judge would see it: the referee records the sequence of
    maze points (junctions, corners, START, box entries) the axle passes
    within route_point_radius_mm of, and sums the true line lengths between
    them. The axle odometer length is still reported (actual_path_mm) but is
    not used, because it depends on the robot's size and turning style.
  * A judge calls a restart when the robot is in FAULT, has left the lines,
    or has not moved for stuck_timeout_s.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..arena import Arena, TruthGraph
from ..config import SimParams
from ..robot.sim_robot import SimRobot
from . import score as S


@dataclass
class MatchConfig:
    power_cycle_between_runs: bool = True
    restart_handling_s: float = 8.0
    checkpoint_radius_mm: float = 60.0
    max_restarts: int = S.MAX_RESTARTS
    stuck_timeout_s: float = 25.0
    off_track_mm: float = 110.0
    route_point_radius_mm: float = 70.0
    shortest_tol_mm: float = 1.0                 # route length rounding tolerance
    penalty_per_restart: float = 0.0
    clamp_negative: bool = False
    skip_actual: bool = False
    total_limit_s: float = S.TOTAL_LIMIT_S       # dry-run cut-off (whole match budget)
    actual_limit_s: float | None = None          # override the computed actual-run budget
    corrupt_storage: bool = False                # flip a flash byte before the actual run
    # fault injection: list of (run, time_s, kind, value)
    #   kind "kick":  rotate the robot by value degrees (simulates a bump)
    #   kind "drag":  shift the robot sideways by value mm (knocked off the line)
    #   kind "jam":   block both wheels (value ignored) - stall detection test
    #   kind "press": operator presses the button (emergency stop)
    #   kind "power": power-cycle the robot (battery connector glitch)
    inject: list = field(default_factory=list)
    trace_every_ms: int = 20


@dataclass
class MatchResult:
    arena: str
    outcome: S.RunOutcome
    score: S.ScoreBreakdown
    true_shortest_mm: float
    actual_path_mm: float | None
    planned_route_mm: float | None
    route_certified: bool
    metrics: dict
    log: list
    trace: list
    final_map: dict


class Match:
    def __init__(self, arena: Arena, params: SimParams, cfg: MatchConfig | None = None):
        self.arena = arena
        self.p = params
        self.cfg = cfg or MatchConfig()
        self.truth = TruthGraph(arena)
        self.true_shortest = self.truth.shortest_length() or float("nan")
        self.sim = SimRobot(arena, params)
        self.sim.core.boot()
        self.rects = arena.rects()[:-1]
        self.outcome = S.RunOutcome()
        self.crossed: list[int] = []
        self.cp_exit_heading: dict[int, float] = {}
        self.trace: list = []
        self.metrics = {
            "false_goal_led": 0,
            "missed_goal_s": 0.0,
            "line_lost_events": 0,
            "restarts_dry": 0,
            "restarts_actual": 0,
            "off_track": 0,
            "stuck": 0,
            "faults": [],
            "dry_path_mm": 0.0,
        }
        self.phase = "setup"
        self.callback = None
        self.callback_every = 16
        self._last_led = False
        self._in_box_ms = 0
        self._route: list = []
        self._dist_cache: dict = {}
        self._pts = np.array(self.truth.points, dtype=float)

    # --------------------------------------------------------------- utils --
    def _dist_to_lines(self, x: float, y: float) -> float:
        r = self.rects
        dx = np.maximum(np.maximum(r[:, 0] - x, x - r[:, 2]), 0)
        dy = np.maximum(np.maximum(r[:, 1] - y, y - r[:, 3]), 0)
        return float(np.min(np.hypot(dx, dy)))

    def _gdist(self, a, b) -> float:
        if a not in self._dist_cache:
            self._dist_cache[a] = self.truth.dijkstra(a)[0]
        return self._dist_cache[a].get(b, float("inf"))

    def route_length(self) -> float | None:
        """True line length of the route recorded during the actual-run
        attempt (consecutive recorded points joined by their shortest
        connection, then on to the nearest box entry)."""
        r = self._route
        if not r:
            return None
        L = sum(self._gdist(a, b) for a, b in zip(r, r[1:]))
        if r[-1] not in self.truth.goal_points:
            L += min((self._gdist(r[-1], g) for g in self.truth.goal_points), default=float("inf"))
        return L

    def _tick(self, run: str, t_run: float) -> None:
        sim = self.sim
        sim.step()
        x, y, th = sim.pose
        if self.cfg.trace_every_ms and (sim.t_us // 1000) % self.cfg.trace_every_ms == 0:
            self.trace.append((sim.t_us / 1e6, x, y, th, run, sim.core.state, int(sim.led)))
        if run == "actual":
            d2 = (self._pts[:, 0] - x) ** 2 + (self._pts[:, 1] - y) ** 2
            k = int(np.argmin(d2))
            if d2[k] < self.cfg.route_point_radius_mm ** 2:
                pt = self.truth.points[k]
                if not self._route or self._route[-1] != pt:
                    self._route.append(pt)
        if run == "dry":
            for i, (cx, cy) in enumerate(self.arena.checkpoints):
                d = math.hypot(x - cx, y - cy)
                if d < self.cfg.checkpoint_radius_mm:
                    if i not in self.crossed:
                        self.crossed.append(i)
                        sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE checkpoint {i + 1} crossed (t_run={t_run:.1f}s)")
                    self.cp_exit_heading[i] = th
        # LED / goal referee
        led = sim.led
        if led and not self._last_led:
            sx = x + self.p.sensor_forward_mm * math.cos(th)
            sy = y + self.p.sensor_forward_mm * math.sin(th)
            if not (self.arena.in_box(sx, sy) or self.arena.in_box(x, y)):
                self.metrics["false_goal_led"] += 1
                sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE LED lit OUTSIDE the end box")
        self._last_led = led
        sx = x + self.p.sensor_forward_mm * math.cos(th)
        sy = y + self.p.sensor_forward_mm * math.sin(th)
        if self.arena.in_box(sx, sy, margin=110) and not led:
            self._in_box_ms += 1
        if self.callback and sim.t_us // 1000 % self.callback_every == 0:
            self.callback(self)

    def _inject(self, run: str, t_run: float, done: set) -> None:
        for k, (r, t, kind, val) in enumerate(self.cfg.inject):
            if k in done or r != run or t_run < t:
                continue
            done.add(k)
            if kind == "kick":
                b = self.sim.body
                b.theta += math.radians(val)
                self.sim.log.append(f"{self.sim.t_us / 1e6:7.3f} SIM injected kick {val} deg")
            elif kind == "drag":
                b = self.sim.body
                b.x += val * math.sin(b.theta)
                b.y -= val * math.cos(b.theta)
                self.sim.log.append(f"{self.sim.t_us / 1e6:7.3f} SIM injected drag {val} mm")
            elif kind == "jam":
                self.sim.body.jammed = True
                self.sim.log.append(f"{self.sim.t_us / 1e6:7.3f} SIM wheels jammed")
            elif kind == "press":
                self.sim.press(0.05)
                self.sim.log.append(f"{self.sim.t_us / 1e6:7.3f} SIM operator pressed the button")
            elif kind == "power":
                self.sim.power_cycle()

    # ----------------------------------------------------------------- run --
    def calibrate(self) -> bool:
        self.phase = "calibration"
        sx, sy, sh = self.arena.start
        self.sim.place(sx, sy, sh)
        self.sim.run_until(lambda: False, 0.2)
        self.sim.press(self.sim.core.get_param("long_press_ms") / 1000 + 0.1)
        ok = self.sim.run_until(lambda: self.sim.core.state in ("IDLE", "FAULT"), 15.0)
        self.sim.place(sx, sy, sh)  # operator re-centres the robot on START
        self.sim.run_until(lambda: False, 0.2)
        return ok and self.sim.core.state == "IDLE"

    def _operator_restart(self, run: str, place) -> None:
        sim = self.sim
        if sim.core.state not in ("FAULT", "IDLE"):
            sim.press(0.05)  # stop it first
            sim.run_until(lambda: sim.core.state == "FAULT", 1.0)
        x, y, h = place
        sim.place(x, y, h)
        sim.run_until(lambda: False, 0.1)
        sim.press(0.06)

    def _restart_pose_dry(self):
        if not self.crossed:
            return self.arena.start
        i = self.crossed[-1]
        cx, cy = self.arena.checkpoints[i]
        th = self.cp_exit_heading.get(i, 0.0)
        # snap to the nearest existing line direction at the checkpoint
        node = min(self.truth.points, key=lambda p: math.hypot(p[0] - cx, p[1] - cy))
        dirs = []
        for nb, _ in self.truth.adj.get(node, []):
            dirs.append(math.degrees(math.atan2(nb[1] - node[1], nb[0] - node[0])))
        want = math.degrees(th)
        best = min(dirs, key=lambda d: abs((d - want + 180) % 360 - 180)) if dirs else want
        return node[0], node[1], best

    def _run_phase(self, run: str, limit_s: float):
        """Runs dry or actual run with restarts. Returns (completed, time_s)."""
        sim = self.sim
        cfg = self.cfg
        t0 = sim.t_us
        extra = 0.0
        injected: set = set()
        anchor = (sim.body.x, sim.body.y, sim.t_us)
        led_time = None
        attempt_odo = sim.body.odometer
        path_mm = None
        while True:
            t_run = (sim.t_us - t0) / 1e6 + extra
            if t_run > limit_s:
                sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE {run} run time limit reached")
                return False, None, None
            self._inject(run, t_run, injected)
            self._tick(run, t_run)
            st = sim.core.state
            if sim.led and led_time is None and st in ("GOAL_CONFIRMATION", "WAIT_FOR_ACTUAL_RUN", "FINISHED",
                                                       "MAP_VALIDATION", "ROUTE_PLANNING"):
                bx, by, _ = sim.pose
                led_time = t_run
                path_mm = sim.body.odometer - attempt_odo
                sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE {run} run complete at {t_run:.2f} s")
            if led_time is not None and st in ("WAIT_FOR_ACTUAL_RUN", "FINISHED", "FAULT", "IDLE"):
                return True, led_time, path_mm
            # judge: restart conditions
            x, y, _ = sim.pose
            reason = None
            if st == "FAULT":
                reason = f"robot fault {sim.core.fault}"
                self.metrics["faults"].append(sim.core.fault)
            elif not self.arena.in_box(x, y) and self._dist_to_lines(x, y) > cfg.off_track_mm:
                reason = "robot left the track"
                self.metrics["off_track"] += 1
            else:
                if math.hypot(x - anchor[0], y - anchor[1]) > 40:
                    anchor = (x, y, sim.t_us)
                elif (sim.t_us - anchor[2]) / 1e6 > cfg.stuck_timeout_s:
                    reason = "robot not making progress"
                    self.metrics["stuck"] += 1
            if reason:
                sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE {run}: {reason}")
                if self.outcome.restarts >= cfg.max_restarts:
                    sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE no restarts left")
                    return False, None, None
                self.outcome.restarts += 1
                self.metrics["restarts_" + run] += 1
                extra += cfg.restart_handling_s
                place = self._restart_pose_dry() if run == "dry" else self.arena.start
                sim.log.append(f"{sim.t_us / 1e6:7.3f} REFEREE restart #{self.outcome.restarts} at {tuple(round(v) for v in place)}")
                self._operator_restart(run, place)
                anchor = (sim.body.x, sim.body.y, sim.t_us)
                attempt_odo = sim.body.odometer
                self._route = []

    def run(self) -> MatchResult:
        sim = self.sim
        cal_ok = self.calibrate()
        if not cal_ok:
            sim.log.append("SIM calibration failed")
        # ---- dry run
        self.phase = "dry"
        sx, sy, sh = self.arena.start
        sim.place(sx, sy, sh)
        sim.press(0.06)
        odo0 = sim.body.odometer
        done, t_dry, _ = self._run_phase("dry", self.cfg.total_limit_s)
        self.metrics["dry_path_mm"] = sim.body.odometer - odo0
        o = self.outcome
        o.checkpoints = len(self.crossed)
        o.dry_completed = done
        o.dry_time_s = t_dry
        if done:
            o.led_glowed = True
        planned = None
        certified = False
        snapshot = sim.core.export()
        if snapshot["route"]["valid"]:
            planned = snapshot["route"]["length"]
            certified = snapshot.get("lower_bound", 0) >= planned - 20
        # ---- actual run
        path = None
        if done and not self.cfg.skip_actual:
            self.phase = "actual"
            sim.run_until(lambda: False, 0.5)
            if self.cfg.corrupt_storage:
                blob = bytearray(sim.core.storage())
                if blob:
                    blob[len(blob) // 2] ^= 0xFF
                    sim.core.set_storage(bytes(blob))
                    sim.log.append(f"{sim.t_us / 1e6:7.3f} SIM corrupted one flash byte")
            if self.cfg.power_cycle_between_runs or self.cfg.corrupt_storage:
                sim.power_cycle()
            sim.place(sx, sy, sh)
            sim.run_until(lambda: False, 0.3)
            sim.press(0.06)
            budget = min(S.actual_time_available(t_dry), S.TOTAL_LIMIT_S - t_dry)
            if self.cfg.actual_limit_s is not None:
                budget = self.cfg.actual_limit_s
            ok, t_act, path = self._run_phase("actual", budget)
            o.actual_completed = ok
            o.actual_time_s = t_act
            if ok:
                o.led_glowed = True
                route = self.route_length()
                self.metrics["route_mm"] = route
                o.shortest_path = route is not None and route <= self.true_shortest + self.cfg.shortest_tol_mm
        sim.log.extend(sim.core.pop_logs())
        self.metrics["line_lost_events"] = sum(1 for l in sim.log if "EVENT line lost" in l)
        self.metrics["missed_goal_s"] = self._in_box_ms / 1000.0
        self.metrics["calibration_ok"] = cal_ok
        self.metrics["map_nodes"] = len(snapshot["nodes"])
        self.metrics["true_junctions"] = len(self.truth.junctions())
        self.metrics["ignored_events"] = sum(1 for l in sim.log if "EVENT ignored" in l)
        self.metrics["loop_closures"] = sum(1 for l in sim.log if "loop closure" in l)
        self.metrics["energy_mAh"] = round(sim.body.energy_mAh, 1)
        sc = S.score(o, self.cfg.clamp_negative, self.cfg.penalty_per_restart)
        return MatchResult(
            arena=self.arena.name,
            outcome=o,
            score=sc,
            true_shortest_mm=self.true_shortest,
            actual_path_mm=path,
            planned_route_mm=planned,
            route_certified=certified,
            metrics=self.metrics,
            log=sim.log,
            trace=self.trace,
            final_map=snapshot,
        )
