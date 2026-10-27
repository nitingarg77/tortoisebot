# T-018: bring `record_capture.sh` (commit ea199de, branch capture-recorder) onto the spike branch
Status: ready
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-018-capture-recorder
Depends on: none (branch `feature/precision-5cm`, owner decision D4)
Design: D4; Risk R7; "Recording design"

## Goal
Get the existing recorder onto the spike branch unchanged, so that T-019 edits a known file and the review can diff it
against what was already field-tested once. No behaviour change in this ticket.

## Acceptance criteria satisfied (AC numbers from the requirements)
Enabling ticket for AC1, AC3 and AC13 (recording with no change to the control launch).

## Files expected to change (max 2 source files + their tests)
- `tortoisebot_firmware/scripts/record_capture.sh` (arrives from `ea199de`)
- Test: `tortoisebot_firmware/test/test_record_capture_syntax.py`

## Approach (step by step)
1. On a branch off `feature/precision-5cm`, `git cherry-pick ea199de`. Do NOT merge the whole `capture-recorder` branch
   and do not resolve conflicts by hand. If the pick touches anything beyond `record_capture.sh` (BUILD_LOG.md entries,
   other files), stop and report which files; BUILD_LOG.md is hook-owned and is not edited by hand. If a conflict occurs
   in `BUILD_LOG.md` only, abort the pick and ask the owner how to bring only the script over.
2. Read the script in full. Write into the test report (not this ticket) a short description of what it actually does,
   checked against design lines 261-273 of BUILD_LOG.md: topics, 30 s silence guard, refuse-overwrite, SIGINT close,
   `ros2 bag info`. Anything the design assumed that the script does not do is listed for T-019.
3. Add `test_record_capture_syntax.py`: runs `bash -n` on the script (subprocess, `shutil.which('bash')` must exist or the
   test fails, not skips), asserts the file is executable and has a `#!` line, and asserts it contains `ros2 bag record`.
4. Do not execute the script, and do not run `ros2`.

## Tests required (and which gates must be seen to fail)
- `bash -n` passes on the script. **Gate seen to fail:** append an unterminated `if` to a COPY of the script in a temp dir
  and confirm the same check fails on it (the test helper takes a path).
- Fast suite count rises by the new tests.

## Follow-ups (found out of scope)
- Behaviour on the robot is only exercised in S0 (T-025), its first deliberate test (R7).

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
