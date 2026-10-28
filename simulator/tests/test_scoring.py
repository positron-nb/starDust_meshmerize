"""Official scoring formula and clock rules (problem statement, pages 4 and 6-7)."""
import pytest

from simulator.scoring import score as S


def test_perfect_run_formula():
    o = S.RunOutcome(checkpoints=4, dry_completed=True, dry_time_s=60, actual_completed=True,
                     actual_time_s=20, shortest_path=True, led_glowed=True)
    b = S.score(o)
    assert (b.A, b.B, b.C, b.S, b.T, b.L, b.P) == (100, 30, 120, 30, 130, 5, 0)
    assert b.total == 100 + 30 + 120 + 30 + 130 + 5


def test_T_and_S_only_on_shortest_path():
    o = S.RunOutcome(checkpoints=2, dry_completed=True, dry_time_s=90, actual_completed=True,
                     actual_time_s=15, shortest_path=False, led_glowed=True)
    b = S.score(o)
    assert b.S == 0 and b.T == 0
    assert b.total == 50 + 30 + 90 + 5


def test_incomplete_dry_run_gets_no_B_or_C():
    o = S.RunOutcome(checkpoints=3, dry_completed=False)
    b = S.score(o)
    assert b.A == 75 and b.B == 0 and b.C == 0 and b.L == 0


def test_checkpoints_counted_once_each():
    # The referee model records each checkpoint index once; the formula is linear.
    assert S.score(S.RunOutcome(checkpoints=4)).A == 4 * S.CHECKPOINT_POINTS


def test_dry_overrun_reduces_actual_time():
    assert S.actual_time_available(150) == 150
    assert S.actual_time_available(180) == 150
    assert S.actual_time_available(200) == pytest.approx(130)  # 20 s overrun deducted
    assert S.actual_time_available(None) == 0


def test_combined_time_limit_constants():
    assert S.DRY_LIMIT_S + S.ACTUAL_LIMIT_S == S.TOTAL_LIMIT_S == 330


def test_negative_C_policy_is_explicit():
    o = S.RunOutcome(dry_completed=True, dry_time_s=200)
    assert S.score(o).C == -20                       # literal reading (default)
    assert S.score(o, clamp_negative=True).C == 0    # alternative reading


def test_penalties_subtract():
    o = S.RunOutcome(checkpoints=1, restarts=2, penalty_points=3)
    b = S.score(o, penalty_per_restart=5)
    assert b.P == 13
    assert b.total == 25 - 13
