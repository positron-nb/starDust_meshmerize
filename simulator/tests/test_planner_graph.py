"""Unit tests of the C++ graph, planner, explorer and persistence code
(the exact code that runs on the ESP32), called through ctypes.

Directions: N=0 E=1 S=2 W=3 in the robot's map frame.
"""
import pytest

from simulator.robot.core_bridge import CoreGraph

J, DEAD, START, GOAL = 0, 1, 2, 3


def ladder():
    """START -> A, then two routes A->B (top 1000 mm, bottom 1400 mm), B -> GOAL.

        T(0,1000)---------U(1000,1000)
          |                    |
     S--A(0,0)---------------B(1000,0)--G
    """
    g = CoreGraph()
    s = g.add_node(0, -300, "N", START)
    a = g.add_node(0, 0, "NES", J)
    b = g.add_node(1000, 0, "NEW", J)
    goal = g.add_node(1300, 0, "W", GOAL)
    g.set_goal(goal)
    g.add_edge(s, "N", a, "S", 300)
    g.add_edge(a, "E", b, "W", 1000)          # direct
    g.add_edge(a, "N", b, "N", 3000, corners=2)  # detour over the top (N, E, S)
    g.add_edge(b, "E", goal, "W", 300)
    return g, s, a, b, goal


def test_validate_clean_graph():
    g, *_ = ladder()
    assert g.validate() == 0


def test_shortest_picks_direct_route():
    g, s, a, b, goal = ladder()
    r = g.plan(s, "N", goal)
    assert r is not None
    assert r["nodes"] == [s, a, b, goal]
    assert r["length"] == pytest.approx(1600)


def test_shortest_with_loop_and_dead_ends():
    # square loop with a dead end; the shorter half of the loop must win
    g = CoreGraph()
    s = g.add_node(0, -200, "N", START)
    a = g.add_node(0, 0, "NESW", J)
    d = g.add_node(-300, 0, "E", DEAD)
    b = g.add_node(500, 0, "NW", J)
    c = g.add_node(0, 800, "ES", J)
    goal = g.add_node(500, 1100, "S", GOAL)
    e = g.add_node(500, 800, "NSW", J)
    g.set_goal(goal)
    g.add_edge(s, "N", a, "S", 200)
    g.add_edge(a, "W", d, "E", 300)
    g.add_edge(a, "E", b, "W", 500)
    g.add_edge(b, "N", e, "S", 800)          # right side: 500 + 800
    g.add_edge(a, "N", c, "S", 800)          # left side: 800 + 500
    g.add_edge(c, "E", e, "W", 500)
    g.add_edge(e, "N", goal, "S", 300)
    assert g.validate() == 0
    r = g.plan(s, "N", goal)
    # both halves of the loop are 1300 mm with 2 turns each: either is optimal
    assert r["length"] == pytest.approx(200 + 1300 + 300)
    assert r["turns"] == 2
    assert d not in r["nodes"]


def test_equal_length_tie_break_by_turns():
    # Two 1000 mm routes from A to B: straight (0 turns) vs a dog-leg (2 corners).
    g = CoreGraph()
    s = g.add_node(0, -100, "N", START)
    a = g.add_node(0, 0, "NES", J)
    b = g.add_node(0, 1000, "NSE", J)
    goal = g.add_node(0, 1200, "S", GOAL)
    g.set_goal(goal)
    g.add_edge(s, "N", a, "S", 100)
    g.add_edge(a, "E", b, "E", 1000, corners=2)  # E, N, W with two corners
    g.add_edge(a, "N", b, "S", 1000)             # straight
    g.add_edge(b, "N", goal, "S", 200)
    assert g.validate() == 0
    r = g.plan(s, "N", goal, turn_weight=30)
    assert [st[1] for st in r["steps"]] == ["N", "N", "N"]
    assert r["turns"] == 0


def test_length_dominates_turn_tiebreak():
    # A route 200 mm shorter but with 4 more turns must still be chosen.
    # straight A->B 1200 mm (1 turn in total) vs dog-leg 1000 mm (3 turns)
    g = CoreGraph()
    s = g.add_node(0, -100, "N", START)
    a = g.add_node(0, 0, "NE", J)
    b = g.add_node(0, 1200, "SNE", J)
    goal = g.add_node(300, 1200, "W", GOAL)
    g.set_goal(goal)
    g.add_edge(s, "N", a, "S", 100)
    g.add_edge(a, "N", b, "S", 1200)
    g.add_edge(a, "E", b, "N", 1000, corners=1)
    g.add_edge(b, "E", goal, "W", 300)
    assert g.validate() == 0
    r = g.plan(s, "N", goal, turn_weight=30)
    assert r["length"] == pytest.approx(100 + 1000 + 300)
    assert r["turns"] == 3


def test_unreachable_goal_returns_none():
    g = CoreGraph()
    s = g.add_node(0, 0, "N", START)
    a = g.add_node(0, 300, "S", J)
    goal = g.add_node(900, 900, "S", GOAL)
    g.set_goal(goal)
    g.add_edge(s, "N", a, "S", 300)
    assert g.plan(s, "N", goal) is None


def test_goal_is_never_routed_through():
    g = CoreGraph()
    s = g.add_node(0, 0, "N", START)
    goal = g.add_node(0, 300, "SN", GOAL)
    x = g.add_node(0, 600, "S", J)
    g.set_goal(goal)
    g.add_edge(s, "N", goal, "S", 300)
    g.add_edge(goal, "N", x, "S", 300)
    r = g.plan(s, "N", x)
    assert r is None


def test_node_matching_radius_and_exits():
    g = CoreGraph()
    n = g.add_node(1000, 500, "NESW", J)
    assert g.match(1040, 520, "NESW", "S", 90) == n          # within radius
    assert g.match(1200, 500, "NESW", "S", 90) == -1         # too far
    assert g.match(1010, 505, "NES", "S", 90) == n           # one exit missed, close -> still matched
    assert g.match(1060, 500, "NES", "S", 90) == -1          # mismatch and not close enough
    g2 = CoreGraph()
    m = g2.add_node(0, 0, "NE", J)
    assert g2.match(0, 0, "NE", "S", 90) == -1               # arrival line must exist at the node


def test_lower_bound_certificate():
    g, s, a, b, goal = ladder()
    # everything explored -> no frontier -> bound is infinite (route proven)
    assert g.lower_bound(s, "N", goal) > 1e20
    g2 = CoreGraph()
    s2 = g2.add_node(0, 0, "N", START)
    a2 = g2.add_node(0, 300, "SEN", J)  # N unexplored
    goal2 = g2.add_node(1300, 300, "W", GOAL)
    g2.set_goal(goal2)
    g2.add_edge(s2, "N", a2, "S", 300)
    g2.add_edge(a2, "E", goal2, "W", 1300)
    lb = g2.lower_bound(s2, "N", goal2)
    assert lb == pytest.approx(300 + 1300)  # frontier at A: d=300, |A-G|=1300


def test_explorer_preference_orders():
    g = CoreGraph()
    s = g.add_node(0, -300, "N", START)
    a = g.add_node(0, 0, "NESW", J)
    g.add_edge(s, "N", a, "S", 300)
    # heading N at A: left = W, straight = N, right = E
    assert g.explore_decide(a, "N", pref=0) == "W"  # LSR
    assert g.explore_decide(a, "N", pref=2) == "N"  # SLR
    assert g.explore_decide(a, "N", pref=5) == "E"  # RSL
    assert g.explore_decide(a, "N", policy=1) == "W"  # left hand


def test_explorer_backtracks_to_nearest_frontier():
    g = CoreGraph()
    s = g.add_node(0, -300, "N", START)
    a = g.add_node(0, 0, "NE", J)       # E unexplored
    d = g.add_node(0, 400, "S", DEAD)
    g.add_edge(s, "N", a, "S", 300)
    g.add_edge(a, "N", d, "S", 400)
    assert g.nearest_frontier(d, "N") == a
    assert g.explore_decide(d, "N") == "S"  # drive back towards A


def test_explorer_reports_done_when_no_frontier():
    g = CoreGraph()
    s = g.add_node(0, 0, "N", START)
    d = g.add_node(0, 300, "S", DEAD)
    g.add_edge(s, "N", d, "S", 300)
    assert g.explore_decide(d, "N") is None


def test_validate_detects_heading_inconsistency():
    g = CoreGraph()
    a = g.add_node(0, 0, "N", J)
    b = g.add_node(0, 500, "E", J)
    # straight edge leaving N must arrive moving N (db = S); db = E is inconsistent
    g.add_edge(a, "N", b, "E", 500, corners=0)
    assert g.validate() >= 1


def test_persistence_roundtrip_and_corruption():
    g, *_ = ladder()
    n = g.persist_roundtrip(-1)
    assert n > 0
    # flip a byte in the payload: CRC must reject the record
    assert g.persist_roundtrip(n - 3) == 0
    # flip a byte in the header magic
    assert g.persist_roundtrip(0) == 0
