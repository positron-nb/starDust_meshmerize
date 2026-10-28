"""Closed-loop simulator tests: the real firmware core drives a simulated
robot on purpose-built arenas. Each test maps to an item of the required
test list (docs/testing_and_validation.md, table "Requirement -> test").

These are SIMULATION results, not hardware validation.
"""
import math

import pytest

from simulator.arena import Arena, GenParams, TruthGraph, generate, sample_arena
from simulator.config import SimParams, harsh, n20_hardware, realistic
from simulator.robot.sim_robot import SimRobot
from simulator.scoring.match import Match, MatchConfig

BOX = 400.0


def arena(name, segs, box, start, cps=()):
    return Arena(name=name, segments=segs, end_box=box, start=start, checkpoints=list(cps))


def straight_arena():
    return arena("straight", [(1150, 300, 1150, 1500)], (950, 1500, BOX, BOX), (1150, 300, 90))


def corner_arena(left=True):
    if left:
        return arena("left_corner", [(1150, 300, 1150, 1000), (1150, 1000, 600, 1000)], (200, 800, BOX, BOX),
                     (1150, 300, 90))
    return arena("right_corner", [(1150, 300, 1150, 1000), (1150, 1000, 1700, 1000)], (1700, 800, BOX, BOX),
                 (1150, 300, 90))


def t_arena():
    # T from the stem; the left arm is a dead end, the right arm leads to the box
    return arena("t_junction", [(1150, 300, 1150, 1000), (600, 1000, 1700, 1000)], (1700, 800, BOX, BOX),
                 (1150, 300, 90), cps=[(1150, 1000)])


def cross_arena():
    # 4-way crossing; left and right arms are dead ends; straight on to the box
    return arena("crossing", [(1150, 300, 1150, 1500), (700, 1000, 1600, 1000)], (950, 1500, BOX, BOX),
                 (1150, 300, 90), cps=[(1150, 1000)])


def two_routes_arena():
    """START -> junction A; A -> B either straight (600 mm) or via a 1600 mm
    detour; B -> box. DFS with straight preference finds the short one; we
    test that whatever is explored, the planned route is the shortest KNOWN."""
    segs = [
        (1150, 200, 1150, 600),     # start stub to A (1150,600)
        (1150, 600, 1150, 1200),    # direct A -> B (1150,1200)
        (1150, 600, 650, 600), (650, 600, 650, 1200), (650, 1200, 1150, 1200),  # detour on the left
        (1150, 1200, 1150, 1500),   # B -> box
    ]
    return arena("two_routes", segs, (950, 1500, BOX, BOX), (1150, 200, 90))


def run(a, p=None, **mc):
    p = p or SimParams(seed=1)
    m = Match(a, p, MatchConfig(**mc))
    return m.run()


def nodes_with_exits(snapshot, n_exits):
    return [n for n in snapshot["nodes"] if bin(n[2]).count("1") == n_exits and n[3] == 0]


# 1. straight line following ------------------------------------------------
def test_straight_line_following_and_goal():
    r = run(straight_arena())
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert r.outcome.shortest_path
    # lateral deviation of the axle from the line stays small
    devs = [abs(x - 1150) for (_, x, y, _, run_, _, _) in r.trace if 400 < y < 1450]
    assert max(devs) < 12.0


# 2/3. 90-degree corners ------------------------------------------------------
@pytest.mark.parametrize("left", [True, False])
def test_corners(left):
    r = run(corner_arena(left), realistic(3))
    assert r.outcome.dry_completed and r.outcome.actual_completed and r.outcome.shortest_path
    # a corner is not a node: START + GOAL only, edge has 2 segments
    snap = r.final_map
    assert len(snap["nodes"]) == 2
    turn = snap["edges"][0]["segs"][0][1]
    assert turn == (1 if left else -1)


# 4. T-junction + 6. dead end ------------------------------------------------
def test_t_junction_and_dead_end_recovery():
    r = run(t_arena(), realistic(4))
    o = r.outcome
    assert o.dry_completed and o.actual_completed and o.shortest_path
    assert o.checkpoints == 1
    assert any("DEAD_END" in l for l in r.log)            # left arm explored first (LSR)
    snap = r.final_map
    assert len(nodes_with_exits(snap, 3)) == 1             # the T
    assert any(n[3] == 1 for n in snap["nodes"])           # a dead-end node


# 5. four-way crossing ---------------------------------------------------------
def test_four_way_crossing():
    r = run(cross_arena(), realistic(5), skip_actual=True)
    assert r.outcome.dry_completed
    snap = r.final_map
    assert len(nodes_with_exits(snap, 4)) == 1


# 7/8. loops and repeated junctions (generated maze with loops) -----------------
def test_loops_and_repeated_junctions():
    a = generate(GenParams(seed=2))  # this maze forces revisits (9 closures in the reference run)
    r = run(a, realistic(6))
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert r.metrics["loop_closures"] >= 1
    assert r.metrics["map_nodes"] <= r.metrics["true_junctions"] + 10   # no node explosion
    assert not r.metrics["faults"]


# 9/10. multiple routes: planned route is the shortest of what was explored ------
def test_multiple_routes_shortest_selected():
    # peek-and-prove mode keeps exploring after seeing the box until the known
    # route is provably shortest, so both routes get mapped and compared
    r = run(two_routes_arena(), realistic(7, firmware={"defer_goal": 1}))
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert r.outcome.shortest_path
    assert r.planned_route_mm == pytest.approx(r.true_shortest_mm, rel=0.04)


# 12/13. sensor noise and surface variation --------------------------------------
def test_sample_arena_harsh_noise():
    r = run(sample_arena(), harsh(8))
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert r.outcome.checkpoints == 4
    assert r.metrics["false_goal_led"] == 0


def test_dull_line_and_shiny_floor():
    p = realistic(9, white_reflectance=0.70, floor_reflectance=0.15, box_reflectance=0.75)
    r = run(sample_arena(), p, skip_actual=True)
    assert r.outcome.dry_completed
    assert r.metrics["false_goal_led"] == 0


# 14. encoder / motor polarity mistakes are caught at calibration -------------------
# BO hardware (default): no encoders, so the gyro checks the calibration pivot.
# N20 hardware: quadrature encoders, whose sign is checked as well.
@pytest.mark.parametrize("hw,fault", [
    ("bo", "motor_polarity_error"),
    ("n20", "encoder_polarity_error"),
    ("n20", "motor_polarity_error"),
])
def test_polarity_mistake_detected(hw, fault):
    kw = n20_hardware() if hw == "n20" else {}
    p = SimParams(seed=10, **kw, **{fault: True})
    m = Match(straight_arena(), p, MatchConfig())
    ok = m.calibrate()
    assert not ok
    assert m.sim.core.fault == "CALIBRATION_BAD"
    assert any("polarity" in l or "not moving" in l or "not rotating" in l or "turned RIGHT" in l for l in m.sim.log)


# optional slot encoders (encoder_mode 1): distance measured, control as without encoders
@pytest.mark.parametrize("profile", [realistic, harsh])
def test_slot_encoder_mode_completes_sample(profile):
    r = run(sample_arena(), profile(31, encoder_mode=1))
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert r.outcome.shortest_path
    assert not r.metrics["faults"]


# 15/16. wheel mismatch and odometry error ------------------------------------------
# 15 % stronger left motor: without encoders the robot must learn the trim
# from its line-following steering.
def test_wheel_mismatch_and_odometry_error():
    p = realistic(11, motor_gain_mismatch=(1.15, 0.85), wheel_diameter_error=(0.03, -0.02))
    a = generate(GenParams(seed=2))
    r = run(a, p)
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert not r.metrics["faults"]


# 17. no false goal at intersections (crossing, no box nearby) -----------------------
def test_no_false_goal_at_crossings():
    r = run(cross_arena(), realistic(12), skip_actual=True)
    assert r.metrics["false_goal_led"] == 0
    assert not any("GOAL false alarm" in l for l in r.log)


# 18/19. genuine end zone: LED on, robot stops fully inside the box ------------------
def test_end_zone_led_and_stop_inside_box():
    a = straight_arena()
    r = run(a, realistic(13))
    assert r.outcome.led_glowed and r.metrics["missed_goal_s"] == 0
    p = SimParams()
    # last trace sample of the actual run: whole body inside the box
    t, x, y, th, *_ = [s for s in r.trace if s[4] == "actual"][-1]
    for d in (p.body_front_mm, -p.body_rear_mm):
        assert a.in_box(x + d * math.cos(th), y + d * math.sin(th))


# 20. line loss and recovery ---------------------------------------------------------
def test_line_loss_recovery_after_bump():
    # A sudden 70 degree bump throws the line out of the array. Encoders do not
    # see an external rotation, so the robot may read it as a line end (U-turn)
    # or as line loss (search sweep); either way it must find the line again
    # and finish without a referee restart.
    r = run(straight_arena(), realistic(14), inject=[("dry", 2.0, "kick", 70.0)], skip_actual=True)
    assert any("EVENT line lost" in l or "DEAD_END" in l for l in r.log)
    assert r.outcome.dry_completed
    assert r.outcome.restarts == 0


# 21/22/23. time limits and combined-time accounting --------------------------------------
def test_dry_run_time_limit():
    r = run(sample_arena(), SimParams(seed=15), total_limit_s=15.0)
    assert not r.outcome.dry_completed
    assert r.score.B == 0 and r.score.C == 0


def test_actual_run_time_limit():
    r = run(sample_arena(), SimParams(seed=16), actual_limit_s=5.0)
    assert r.outcome.dry_completed
    assert not r.outcome.actual_completed
    assert r.score.S == 0 and r.score.T == 0


# 24. restarts: from the last checkpoint, max three, clock keeps running -----------------
@pytest.mark.parametrize("press_s", [15.0, 35.0])
def test_dry_restart_from_checkpoint_relocalises(press_s):
    # judge-called restart: the operator stops the robot, places it on the last
    # checkpoint crossed (START if none yet - the 15 s case) and presses START
    r = run(sample_arena(), realistic(17), inject=[("dry", press_s, "press", 0)])
    o = r.outcome
    assert o.restarts == 1
    assert o.dry_completed and o.actual_completed
    assert any("RELOC aligned" in l for l in r.log)
    assert o.shortest_path
    # restart handling time is included in the dry-run time
    assert o.dry_time_s > 36.0 + MatchConfig().restart_handling_s - 5


def test_restart_limit_three():
    inj = [("dry", t, "press", 0) for t in (5.0, 20.0, 35.0, 50.0)]
    r = run(sample_arena(), SimParams(seed=18), inject=inj)
    assert r.outcome.restarts == 3
    assert not r.outcome.dry_completed
    assert any("no restarts left" in l for l in r.log)


def test_displaced_robot_detects_inconsistent_map():
    # bumped sideways onto another line WITHOUT a restart: the map stops
    # matching; the robot must notice, roll back and re-localise instead of
    # circling and creating duplicate nodes until the time runs out
    r = run(sample_arena(), realistic(17), inject=[("dry", 15.0, "drag", 150.0)], skip_actual=True)
    assert r.metrics["map_nodes"] < 40
    assert any("MAP inconsistent" in l or "RELOC aligned" in l for l in r.log) or r.outcome.dry_completed


# 25/26. route persistence across a power cycle; corrupted storage -----------------------
def test_route_survives_power_cycle():
    r = run(t_arena(), SimParams(seed=19), power_cycle_between_runs=True)
    assert any("STORE loaded: stage=2" in l for l in r.log)
    assert r.outcome.actual_completed and r.outcome.shortest_path


def test_corrupted_storage_falls_back_safely():
    r = run(t_arena(), SimParams(seed=20), corrupt_storage=True)
    assert any("CRC mismatch" in l or "no valid record" in l for l in r.log)
    # stage is lost -> the robot treats the press as a new dry run / search; it must
    # still reach the box without crashing or lighting the LED elsewhere
    assert r.outcome.actual_completed
    assert r.metrics["false_goal_led"] == 0


# 27. checkpoint scoring on the reconstructed sample arena --------------------------------
def test_sample_arena_checkpoints_and_score():
    r = run(sample_arena(), SimParams(seed=21))
    assert r.outcome.checkpoints == 4
    assert r.score.A == 100
    assert r.outcome.shortest_path
    assert r.score.total > 400


# 29. different topologies ------------------------------------------------------------------
@pytest.mark.parametrize("gp", [
    GenParams(seed=31, loop_fraction=0.0),                      # tree maze, no loops
    GenParams(seed=32, loop_fraction=0.5),                      # many loops
    GenParams(seed=33, cols=5, rows=5, min_spacing=300),        # sparse, long lines
    GenParams(seed=34, cols=8, rows=8, min_spacing=190),        # dense
])
def test_generated_topologies(gp):
    a = generate(gp)
    assert not a.validate()
    r = run(a, realistic(gp.seed))
    assert r.outcome.dry_completed and r.outcome.actual_completed
    assert not r.metrics["faults"]


# 30. emergency stop and stall detection ------------------------------------------------------
def test_operator_emergency_stop():
    a = straight_arena()
    sim = SimRobot(a, SimParams(seed=22))
    sim.core.boot()
    sim.press(1.6)
    sim.run_until(lambda: sim.core.state == "IDLE", 10)
    sim.place(*a.start)
    sim.press(0.05)
    sim.run_until(lambda: sim.core.state == "DRY_RUN", 2)
    sim.run_until(lambda: False, 1.0)
    sim.press(0.05)
    assert sim.core.state == "FAULT"
    sim.step()
    assert sim.out.duty_left == 0 and sim.out.duty_right == 0


# BO: no encoders - detected as too much modelled travel without a junction.
@pytest.mark.parametrize("hw", ["bo", "n20"])
def test_stall_detected_and_motors_stopped(hw):
    kw = n20_hardware() if hw == "n20" else {}
    m = Match(straight_arena(), SimParams(seed=23, **kw), MatchConfig(inject=[("dry", 2.0, "jam", 0)], max_restarts=0))
    r = m.run()
    assert "STALL" in r.metrics["faults"]
    assert m.sim.out.duty_left == 0 and m.sim.out.duty_right == 0


def test_truth_graph_sample_shortest_length():
    # independent Python Dijkstra on the reconstructed sample arena
    assert TruthGraph(sample_arena()).shortest_length() == pytest.approx(6580.0)
