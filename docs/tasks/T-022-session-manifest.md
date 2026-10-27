# T-022: session manifest writer (what actually landed, not what was asked for)
Status: ready
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-022-session-manifest
Depends on: none
Design: "Bag storage", `manifest.yaml`; "Data model"; CLAUDE s1 ("check the value landed")

## Goal
A script that records a session's facts into `manifest.yaml`: robot commit, launch args, the Cartographer config basename as
read from the Cartographer LOG, `camera_ros` version, live `jpeg_quality`, `df` before and after, clock sync state. The value
read from the log is never defaulted to the value requested.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC13 (confirm the control launch is unchanged and which config loaded).
- AC1 (disk and timing facts for the report), AC3 (one manifest per pass).

## Files expected to change (max 2 source files + their tests)
- `tortoisebot_firmware/tortoisebot_firmware/session_manifest.py` (new; pure parsers and the dict builder)
- `tortoisebot_firmware/scripts/session_manifest.py` (new; thin shell with `start` and `finish` subcommands)
- Test: `tortoisebot_firmware/test/test_session_manifest.py`

## Approach (step by step)
1. Pure functions:
   ```python
   def parse_carto_config_basename(log_text: str) -> str | None       # None when not found; NEVER a default
   def parse_timedatectl_show(text: str) -> dict                       # NTPSynchronized, TimeUSec, Timezone
   def parse_df_avail_kb(text: str) -> int
   def parse_ros2_param_get(text: str) -> str | None                   # "Integer value is: 80"
   def build_manifest(start: dict, finish: dict, operator_notes: str) -> dict   # keys per the design Data model
   ```
2. The Cartographer log line format is not known from here. Implement against a sample line copied from a real log on the
   robot (the implementer asks the owner for one, or the ticket stays blocked on "sample line"); until then the parser looks for
   the string `slam_real` followed by `.lua` and returns the basename, and the test says so. Mark the regex `unverified`
   in the docstring until S0 confirms it against a real log.
3. Shell: `session_manifest.py start --dir D --session S1 --type S1 --pass 1 --launch-args "..."` writes `manifest.yaml`
   partial; `finish` fills end time, `df` after, clock state, notes. It shells out only to `git rev-parse HEAD` (in the robot
   checkout), `timedatectl show`, `df`, `ros2 param get /camera_node jpeg_quality`, `dpkg -l ros-humble-camera-ros`; each
   failure leaves the field `null` with an `errors` list. No ROS imports.
4. YAML written with the `yaml` module if importable, otherwise a tiny hand writer; state which in the docstring.

## Tests required (and which gates must be seen to fail)
- Canned log containing `slam_real_rigid.lua` -> `slam_real_rigid.lua`; a log with no mention -> `None`; **gate seen to
  fail:** make the parser return a default `slam_real.lua` on a miss and the miss test must fail.
- Canned `timedatectl show` -> `NTPSynchronized` parsed; `df` text parsed; `ros2 param get` text parsed; garbage -> None.
- `build_manifest` has every key listed in the design Data model row for `manifest.yaml`; a missing finish step yields a
  manifest marked `complete: false`.
- Shell test with all external commands replaced by fakes on PATH: manifest written, nothing raised when a command is absent.

## Follow-ups (found out of scope)
- Regex against a real Cartographer log is confirmed in S0.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
