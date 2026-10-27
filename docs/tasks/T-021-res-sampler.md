# T-021: `res_sampler.py` (CPU cores, load, temperature, throttled flags, disk, every 2 s)
Status: ready
Type: code
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-021-res-sampler
Depends on: none
Design: "Pi cost budget and how it is measured"; "On the Pi" row `res_sampler.py`; "Interfaces" row "Resource sampling (Pi)"

## Goal
A no-ROS sampler that writes `resources.csv` so the cost of recording (AC2) is measured, not estimated. The arithmetic is pure
and tested on canned `/proc` text; the shell reads files and runs `vcgencmd`.

## Acceptance criteria satisfied (AC numbers from the requirements)
- AC1 (CPU/load average and temperature on the Pi while recording, with n of samples and spread).
- AC2 (the cost over the baseline, measured on the same stack).

## Files expected to change (max 2 source files + their tests)
- `tortoisebot_firmware/tortoisebot_firmware/proc_sample.py` (new; pure)
- `tortoisebot_firmware/scripts/res_sampler.py` (new; shell)
- Test: `tortoisebot_firmware/test/test_proc_sample.py`

## Approach (step by step)
1. Pure API:
   ```python
   @dataclass(frozen=True) class Snapshot: wall_ns: int; mono_ns: int; loadavg1: float | None; temp_c: float | None; throttled_hex: str; mem_avail_kb: int | None; disk_free_kb: int | None; proc_ticks: dict[str, tuple[int,int] | None]   # name -> (utime, stime) or None if the pid is gone
   def parse_proc_stat(text: str) -> tuple[int, int]            # fields 14 and 15 of /proc/<pid>/stat; the comm field may contain spaces and parentheses, so split after the LAST ')'
   def parse_loadavg(text: str) -> float
   def parse_meminfo_available(text: str) -> int
   def parse_vcgencmd_temp(text: str) -> float                   # "temp=62.7'C"
   def parse_vcgencmd_throttled(text: str) -> str                # "throttled=0x50000" -> "0x50000"
   def cores_between(a: Snapshot, b: Snapshot, clk_tck: int, name: str) -> float | None   # (d_utime + d_stime) / clk_tck / d_mono_seconds; None if either is None
   def total_busy_cores(a_stat: str, b_stat: str, ...)   # whole-Pi from /proc/stat line "cpu"; document the choice
   ```
   A missing field is empty (None -> empty CSV cell), never 0.
2. CSV columns exactly as the design: `wall_ns, mono_ns, arm, loadavg1, temp_c, throttled_hex, mem_avail_kb, disk_free_kb`, then
   `<name>_utime,<name>_stime` per named PID. `--arm A0|A1|B|rest` labels rows; `--pid name=PID` repeated; `--interval 2.0`;
   `--out PATH`; refuses to overwrite.
3. The shell runs itself at `os.nice(10)` on start, reads `os.sysconf('SC_CLK_TCK')`, and handles `FileNotFoundError` for an
   exited PID (cell empty, keep sampling). `vcgencmd` missing -> empty cells. fsync per line.
4. Sampling must cost almost nothing: file reads only, one `vcgencmd` pair per sample; document the measured cost in S1 (not here).

## Tests required (and which gates must be seen to fail)
- Canned `/proc/<pid>/stat` text including a comm like `(Web Content (x))`: ticks parsed correctly (gate: split on the first ')'
  and the test must fail).
- `cores_between` on two canned snapshots 2.0 s apart with 100 ticks of utime+stime at CLK_TCK 100 gives 0.5 cores.
- A PID that exited between samples gives an empty cell and no exception; first sample after has the PID back only if present.
- `parse_vcgencmd_throttled("throttled=0x50005")` is `0x50005`; garbage text yields None, not 0.
- CSV header matches the design exactly; the shell run with `--dry-run` or a fake-clock test writes the header plus rows.

## Follow-ups (found out of scope)
- Whether to nice the recorder is decided from S1 data (design), not here.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
