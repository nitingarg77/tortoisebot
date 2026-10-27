# T-014: report rendering and the (blank) thresholds file
Status: blocked (T-002, T-009)
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-014-spike-report
Depends on: T-002, T-009
Design: "Comparisons" last paragraph; "Pure modules" `spike_report`; test-strategy line on thresholds

## Goal
Markdown tables for the spike report, which print "not judged" while the thresholds file is blank and pass/fail once the owner
fills numbers in, with no code change.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC9 (thresholds are owner numbers; blank means the numbers are printed and not judged).
- AC11 (report ends with a result line, a "Not verified" section and a recommendation; does not claim 5 cm).
- AC7, AC8 (tables per pass, before and after alignment next to each other).

## Files expected to change (max 2 source files + their tests)
- `analysis/camera_spike/camera_spike/spike_report.py` (new)
- `analysis/camera_spike/thresholds.yaml` (new; keys present, all values null, with a comment line per key)
- Test: `analysis/camera_spike/test/test_spike_report.py`

## Approach (step by step)
1. `thresholds.yaml` keys: `mean_error_cm_max`, `worst_error_cm_max`, `rest_step_cm_max`, `tracking_loss_s_max`,
   `cpu_extra_cores_max` (the last pre-filled with 0.25, owner decision Q6; the rest null).
2. Signatures:
   ```python
   def load_thresholds(path: Path) -> dict[str, float | None]
   def judge(metric_name: str, value: float, thresholds) -> str     # 'not judged' | 'pass' | 'fail'
   def table_per_cross(rows: list[dict]) -> str                       # cross, mark times, window, error U0/U1/F-carto/F-tape-all/F-tape-LOO, Cartographer error
   def table_stats(title: str, stats: dict[str, Stats], thresholds) -> str
   def render_report(sections: dict, thresholds, recommendation: str) -> str     # ends with "Result:", "## Not verified", "Recommendation:"
   ```
3. `recommendation` must be one of `tracks well | does not track | inconclusive`; anything else raises `SpikeError`.
   `render_report` refuses (raises) if the "Not verified" list is empty: a report with nothing unverified is not honest.
4. The renderer never prints the words "5 cm met" or "meets 5 cm"; add a function `assert_no_5cm_claim(text)` used by the
   renderer and by the test.
5. Tables show the "tape floor: a few cm (Q5)" line at the top of every error table.

## Tests required (and which gates must be seen to fail)
- With the shipped `thresholds.yaml`: every judged cell prints `not judged`.
- With thresholds filled (temp file): the same values print `pass` and `fail` at the right side of each limit.
  **Gate seen both ways (design requirement):** make `judge` return 'pass' for blank thresholds and the first test must fail;
  make it ignore filled values and the second must fail.
- The report ends with `Result`, `Not verified` and `Recommendation`; an invalid recommendation raises; an empty Not-verified
  list raises; text containing "meets 5 cm" fails `assert_no_5cm_claim`.
- `cpu_extra_cores_max = 0.25` judges 0.30 as fail and 0.20 as pass.

## Follow-ups (found out of scope)
- Figures (plots) are made in T-034 using matplotlib if present on the box; this ticket is tables only.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
