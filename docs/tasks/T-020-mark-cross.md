# T-020: `mark_cross.py` (owner presses Enter at each cross arrival and departure)
Status: ready
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-020-mark-cross
Depends on: none (the CSV fields match `Mark` in T-002 by convention; nothing is imported)
Design: "Marking cross arrivals (AC4)", source 1; "On the Pi" table row `mark_cross.py`

## Goal
A no-ROS terminal tool for the Pi that writes `marks.csv`, one flushed and fsynced line per event, with both wall and monotonic
clocks, so marks and bag share a clock.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC4 (arrival time of each cross marked in a side file; the matching error is reported later from these lines).
- AC12 (publishes nothing, adds no motion).

## Files expected to change (max 2 source files + their tests)
- `tortoisebot_firmware/tortoisebot_firmware/mark_logic.py` (new; pure; no ROS, no `time` calls inside logic)
- `tortoisebot_firmware/scripts/mark_cross.py` (new; thin shell)
- Test: `tortoisebot_firmware/test/test_mark_logic.py`

## Approach (step by step)
1. Read `tortoisebot_firmware/tortoisebot_firmware/motor_math.py` and `scripts/differential.py` for the logic/shell split.
2. Read the Set P order from `docs/test-reports/run4-precision-5cm-plan.md` section 2 (crosses p1 to p10 per the design);
   make the order a tuple constant `SET_P_ORDER` and cite the section in a comment. Do not invent coordinates here.
3. Pure API:
   ```python
   @dataclass(frozen=True) class MarkState: index: int; expecting: str   # expecting in {'arrive','leave'}; index into SET_P_ORDER
   def next_event(state: MarkState, line: str, wall_ns: int, mono_ns: int) -> tuple[MarkState, Record | None]
   def format_csv_line(r: Record) -> str     # wall_ns,mono_ns,label,event,text
   CSV_HEADER = "wall_ns,mono_ns,label,event,text"
   ```
   Enter on empty line advances: arrive then leave per cross. `n <text>` records a note (no advance). `u` undoes the last
   event by emitting a `note` "undo" (nothing is erased; append-only). After the last `leave` further Enter lines do not invent
   a p11: they return a note "extra enter ignored".
4. The shell: `python3 scripts/mark_cross.py --out DIR/marks.csv`; refuses to overwrite an existing file; opens in append
   mode, writes the header once, `flush()` + `os.fsync()` per line; clocks `time.time_ns()` and `time.monotonic_ns()` read
   immediately after the line is received; prints the label expected next. Adds `start`/`stop` events via `s` and `e` for S5.
5. Exits cleanly on EOF/Ctrl-C (file closed).

## Tests required (and which gates must be seen to fail)
- Full pass of 10 crosses via `io.StringIO` with a fixed clock: 20 records in Set P order, arrive/leave alternating.
- Notes do not advance the state; the extra Enter at the end does not create `p11` (**gate seen to fail:** remove the end guard,
  the test must fail).
- `format_csv_line` escapes commas and newlines in notes (quote or replace; state which).
- The shell, run as a subprocess with stdin piped, writes a file whose header and line count are as expected and which is
  complete even if the process is killed after the 3rd line (write the file, kill with SIGKILL, read 3 lines).
- Refuses to overwrite an existing file (non-zero exit).

## Follow-ups (found out of scope)
- Key-press lag is measured by comparing marks to stationary windows (T-006) in S4; not a desk test.
- If the `ssh` drops, marks stop; the design accepts this.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
