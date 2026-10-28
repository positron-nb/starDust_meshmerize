"""Official Meshmerize 2026-27 scoring (problem statement, "Judging" and
"Scoring" sections), reproduced term by term:

  A = 25 x (number of checkpoints covered during the Dry Run)
  B = 30 if the bot successfully completes the Dry Run
  C = 180 - total time taken in seconds to complete the Dry Run
  S = 30 if the bot successfully completes through the Shortest Path
  T = 150 - total time taken to complete the Actual Run
      (only if the bot completes in the shortest path)
  L = 5 if the LED glows
  P = penalties
  Total = (A + B + C + S + L + T) - P

AMBIGUITIES (not resolved by the PDF; defaults chosen here are flagged and
configurable - see docs/competition_rules.md for the questions to ask):
  * C when the dry run is not completed, or takes longer than 180 s: the
    PDF does not say whether C is 0, negative, or capped.  Default: C is
    computed only for a completed dry run and may be negative
    (`clamp_negative=False`); B and C are 0 for an incomplete dry run.
  * T for a completion time above 150 s (possible only if overrun rules
    allow): default computed literally (may be negative).
  * P: no penalty values are defined in the PDF. Default 0 per restart,
    configurable.
  * L: awarded once if the LED glows at the end zone in either run.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

DRY_LIMIT_S = 180.0
ACTUAL_LIMIT_S = 150.0
TOTAL_LIMIT_S = 330.0
CHECKPOINT_POINTS = 25
DRY_COMPLETE_POINTS = 30
SHORTEST_POINTS = 30
LED_POINTS = 5
MAX_RESTARTS = 3


@dataclass
class RunOutcome:
    checkpoints: int = 0
    dry_completed: bool = False
    dry_time_s: float | None = None
    actual_completed: bool = False
    actual_time_s: float | None = None
    shortest_path: bool = False
    led_glowed: bool = False
    restarts: int = 0
    penalty_points: float = 0.0


@dataclass
class ScoreBreakdown:
    A: float
    B: float
    C: float
    S: float
    T: float
    L: float
    P: float
    total: float

    def as_dict(self):
        return asdict(self)


def score(o: RunOutcome, clamp_negative: bool = False, penalty_per_restart: float = 0.0) -> ScoreBreakdown:
    A = CHECKPOINT_POINTS * o.checkpoints
    B = DRY_COMPLETE_POINTS if o.dry_completed else 0.0
    C = 0.0
    if o.dry_completed and o.dry_time_s is not None:
        C = DRY_LIMIT_S - o.dry_time_s
        if clamp_negative:
            C = max(0.0, C)
    S = 0.0
    T = 0.0
    if o.actual_completed and o.shortest_path and o.actual_time_s is not None:
        S = SHORTEST_POINTS
        T = ACTUAL_LIMIT_S - o.actual_time_s
        if clamp_negative:
            T = max(0.0, T)
    L = LED_POINTS if o.led_glowed else 0.0
    P = o.penalty_points + penalty_per_restart * o.restarts
    total = A + B + C + S + L + T - P
    return ScoreBreakdown(A, B, C, S, T, L, P, total)


def actual_time_available(dry_time_s: float | None) -> float:
    """Rule: 'If a team fails to complete a dry run in 3 min, extra time
    will be deducted from the time period of the actual run'."""
    if dry_time_s is None:
        return 0.0
    return ACTUAL_LIMIT_S - max(0.0, dry_time_s - DRY_LIMIT_S)
