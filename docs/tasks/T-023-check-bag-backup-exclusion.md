# T-023: check that spike bags and datasets are excluded from the Drive backup (D7)
Status: ready (read-only box check; needs the owner's go because it is a box job)
Type: job
Feature: camera-assisted-localisation (Phase 1 spike)        Branch: feature/T-023-backup-exclusion
Depends on: none (must be DONE before the first bag exists: T-025 depends on this)
Design: D7; "Security"; Risk R10

## Goal
Establish, by reading, whether the paths that will hold spike bags and datasets are inside the box's daily Google Drive backup
(rclone, 03:00, `/workspace/tools/gdrive-backup`). The design says this was NOT checked. Bags contain images of the building
and the people in it; the default is an excluded path.

## Acceptance criteria satisfied (AC numbers from the requirements)
Privacy gate for AC1/AC3 (no AC number covers it; design "Security" and D7). Nothing is recorded until this has a result.

## Files expected to change (max 2 source files + their tests)
- None in the repo. The result is written to `docs/test-reports/camera-spike-backup-check.md` (new report).
- If an exclusion line has to be added, that is a change to the box backup tool outside this repo: STOP and ask the owner; this
  ticket does not edit it.

## Approach (step by step)
1. Job rules: box, read-only, no heavy work. Do not run the backup. Read the config only.
2. Read `/workspace/STORAGE.md` and the backup tool's include/exclude list under `/workspace/tools/gdrive-backup` (and the
   Drive memory note `gdrive-backup.md`). Note the symlink trap: trailing-slash patterns do not match symlinked directories.
3. Decide the two paths: the box landing directory for copied bags (suggested `/workspace/robotics/camera_spike_data/`) and the
   Pi `~/captures/` (Pi is not in the Drive backup unless the tool says otherwise).
4. State, for each path, "in backup / not in backup / unknown", with the exact pattern that decides it and the file read.
5. Report to the owner: accept backup of the bags, or request an exclusion. Record the owner's answer in the report.

## Tests required (and which gates must be seen to fail)
- No code. The check is a reading; the report quotes the lines read. "Gate": a dry-run listing (`rclone ... --dry-run` with
  the tool's own filter flags, if the tool supports it) showing the data path is or is not listed, run only if the owner
  agrees and as a light job.

## Follow-ups (found out of scope)
- If the data must live somewhere else, the paths in T-015 / T-031 / T-034 change; the owner decides.

## Blockers (filled by /implement after 3 failed attempts)

## Review (filled by /review)
