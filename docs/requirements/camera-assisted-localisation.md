# Requirements: camera-assisted localisation
Status: ready for owner scope approval (open-question defaults accepted 2026-10-27, see "Owner decisions")
Owner approval: pending

## Owner decisions (2026-10-27)

The owner replied "default" to questions 1 to 4 and 6, so the recommended default of each stands:

- Q1: the camera must fix C (independent stop-pose measurement) and B (drift while driving); A only if T-001 shows jumps persist.
- Q2: T-001 runs first.
- Q3: AC9 thresholds stay blank; the spike reports numbers and the pass line is agreed after seeing them.
- Q4: at least 3 Set P passes, a 10 minute rest recording, 5 guarded 1 m drives.
- Q6: at most 0.25 extra Pi cores while recording.
- Q5 and Q7 to Q14: not yet discussed; their defaults are proposals only.

Sources are cited as [file:line or section]. Short names: FINDINGS = `docs/test-reports/run4-localisation-findings.md`; T-001 = `docs/tasks/T-001-rigid-frame-slam.md`; PLAN = `docs/test-reports/run4-precision-5cm-plan.md`; CLAUDE = `CLAUDE.md`. Statements without a source are in Open questions, not here.

## Problem
The Phase 1 target is every goal reached within 5 cm (D-031, D-032) [PLAN:3-4]. On 2026-10-27 the localiser, not Nav2, stopped that being checked [FINDINGS:3-6]:
- The SLAM `map` frame jumped 0.41 m in y with the robot at rest on (0, 0); Cartographer logged loop-closure constraints disagreeing with the local estimate by 0.41 m, then 0.14 m [FINDINGS "The jump"].
- After a hand move the pose took about 1 to 4 min to settle; cross 3 first read (0.99, 0.79) against a plan of (1.00, 0.50) [FINDINGS Table 3, "Plan corrections"].
- AMCL on a hand-saved map did not converge: std dev 0.26 to 0.41 m, one reading each, so observations not measurements [FINDINGS Table 2].
- Rest drift in SLAM mode was 1.6 to 2.6 cm max spread over 5 runs of 60 s [FINDINGS Table 1].
- Nav2's 0.03 m tolerance and the other changes were never exercised, because no Set P goal was run [FINDINGS "Not verified"].

Current sensing: Cartographer uses lidar + IMU only (`num_laser_scans = 1`, `use_imu_data = true`, `use_odometry = false`) [tortoisebot_slam/config/slam_real.lua:37,40,70]; there are no wheel encoders [CLAUDE intro]. The camera is tilted 4.5 deg down and only feeds floor-obstacle detection merged into `/scan_fused` for the costmaps [BUILD_LOG.md:329-334; `scan_fusion.py`, `floor_scan.py`; BUILD_LOG.md:1007]. It contributes nothing to the pose today.

Pending input: T-001 (Cartographer global loop closure off, measure-only) has not been run on the robot [T-001 status line]. Its result decides how much a camera must fix.

## Users
- Owner/operator: runs Set P and exploration at the robot, needs a pose that can be judged against floor tape.
- Developer/QA agents: need numbered, measurable criteria that tickets can cite.

## User stories
- As the owner, I want to know whether the camera can track the robot's motion well enough to improve the pose, so that I decide on evidence whether to build live use or switch to fiducial tags.
- As the owner, I want the robot's control loop to keep working when the box or Wi-Fi is gone, so that a localisation add-on never makes the robot less safe than today.
- As the developer, I want the experiment to be offline and non-moving-by-itself, so that it costs the Pi nothing it cannot afford.

## Acceptance criteria
Which failure the camera must fix (A, B or C below) is not decided; see Open question 1. Criteria are grouped so the owner can approve those that apply.

### Phase 1: offline spike (the only phase in scope)
Reference truth: the taped floor crosses of Set P, placed by hand to "a few cm" [FINDINGS Table 3 note]. That placement error is a floor under any comparison and must be stated in the result.

Recording
- AC1: A rosbag is recorded on the robot during a Set P pass containing camera images, IMU, lidar `/scan`, `/tf` and `/tf_static`, at rates chosen to be modest. The report states, for every topic, rate, size and the CPU/load average and temperature on the Pi while recording, measured, with n of samples and spread (CLAUDE s1, s5).
- AC2: The Pi-side recording adds no more than a limit to be set by the owner (Open question 6) over a baseline measured on the same stack without recording. Until the owner sets it, the report states the measured cost and no pass/fail is claimed.
- AC3: The recording is a single bag per pass, with the owner present; the pass is driven by the existing Set P method (hand placement or the guarded tools), not by new code (see AC12).
- AC4: The time of each cross arrival is marked in the bag or a side file, so trajectory samples can be matched to crosses. The report states how time was matched and its error.

Comparison (on the box, offline, one heavy job at a time, owner asked first [CLAUDE "Commands"])
- AC5: Visual SLAM is run offline on the bag. The report names the system and version used (this is a design/experiment choice, recorded as found, not fixed here).
- AC6: The visual trajectory is compared with the taped crosses (position error per cross, in cm), and with the Cartographer pose recorded in the same pass.
- AC7: The comparison covers at least the number of passes set in Open question 4, each reported separately, with n, mean and spread of per-cross error (CLAUDE s1: one run is not a measurement). Any pass where visual tracking was lost is reported with where and for how long, not dropped.
- AC8: Scale handling is stated: whether the visual trajectory needed scale or alignment fitted to the tape or the lidar, and how; error after fitting is reported next to error before, so a fit cannot hide a poor result.

"Camera tracks well" (measurable; thresholds to be confirmed by the owner, Open question 3)
- AC9: Candidate definition, per pass: visual tracking is not lost during the pass; the per-cross position error (after the fit in AC8) has a mean and a worst case under values set by the owner; the visual trajectory has no step above a value set by the owner while the robot is at rest. The thresholds are numbers the owner fills in; without them the spike reports the numbers and does not declare pass or fail.
- AC10: Which criterion applies depends on the failure chosen in Open question 1:
  - A (frame jump at rest, AMCL spread): the visual trajectory at rest shows no step above the AC9 value across at least the rest duration in Open question 4, and the report states whether it stays steady through the interval where the Cartographer frame jumped on 2026-10-27 type events (if one occurs in the recording; if none occurs, say so, it cannot be claimed).
  - B (drift while driving, no encoders): drift per metre over guarded 1 m drives, n of at least the number set in Open question 4, mean and spread, compared with Cartographer's figure from T-001 criterion 5.
  - C (independent measurement of where the robot stopped for the 5 cm test): per-cross error at stop against the tape, with the tape placement error from the floor stated, and an explicit statement of whether the camera's error is small enough to distinguish 5 cm from 10 cm given that tape error.
- AC11: The report ends with a result line, a "Not verified" section, and a stated recommendation among: camera tracks well (go to Phase 2 scope), camera does not (fiducial tags, new requirements), inconclusive (what to measure next). It does not claim 5 cm.

Safety
- AC12: Phase 1 adds no code that publishes `cmd_vel` or commands motion. If a drive is needed for B, it uses an existing guarded tool; any new tool that commands motion must have a clearance guard, a did-it-move check, a stop on every exit path and a leash sized to the job, and stopping means cancelling the goal or deactivating the holder node, not killing the client [CLAUDE s2]. Owner is present at the robot [CLAUDE "Hardware boundary"].
- AC13: Recording does not change what the robot runs for control: the lidar + IMU Cartographer loop and Nav2 start from the same launch as before, with the value confirmed to be live after any change (CLAUDE s1), and the baseline for AC2 is taken the same way.

Resources and fallback
- AC14: Heavy visual SLAM runs only on the box, never on the Pi. Before the run, the owner is asked and the job is a `job` ticket; it is one heavy job at a time (shared machine) [CLAUDE "Commands"].
- AC15: With the box or Wi-Fi absent during Phase 1, the robot is unaffected, because Phase 1 has no live dependency. The report confirms this by showing the control loop's behaviour with the bag copy not yet made (the bag is recorded locally on the Pi, copied afterwards).

### Phase 2: live hybrid (out of scope unless Phase 1 passes)
Not approved by this document. Recorded so the design stage knows the owner's intent. If the owner approves scope after Phase 1, a separate requirements revision must say, at least:
- The Pi keeps the lidar + IMU control loop; the box sends only low-rate absolute corrections, applied smoothly and ignored if late (owner decision, this task).
- The box and Wi-Fi are not in the live control loop (Cartographer moved off the Pi over Wi-Fi made the pose run away on 2026-09-28, scans about 90 ms late with jitter: owner-supplied, source not found in the repo, see Open question 7; BUILD_LOG.md:155 records a runaway-on-late-scans fault on 2026-09-25).
- Acceptance numbers for latency, correction size, behaviour on loss of link, and Pi CPU, each measured with n, mean, spread.

## Explicitly out of scope
- Any live use of the camera for pose, in this document (Phase 2).
- A design or choice of technology: visual SLAM system, tags, or otherwise (next stage, owner-approved).
- Changing `slam_real.lua`, AMCL or Nav2 parameters, or any code on the robot beyond what AC1 needs to record (recording tooling only).
- Claiming the 5 cm target is met; that is Set P's job [FINDINGS "Not verified"].
- Wheel encoders or hardware changes (D-031: no encoders in Phase 1 [PLAN:3-4]); the fan suggestion [T-001 Follow-ups].
- Fixing AMCL, the explorer supervisor stall, or the pose settling time, except as measured by the comparison.
- Camera-based obstacle detection (`/scan_fused`) changes.
- Running T-001; it is separate, and the spike may run before or after it (Open question 2).

## Links to program layer (SRD item, risk-register id)
- No SRD or risk register exists in this repo (`docs/` holds only WORKFLOW, templates, test-reports, tasks). No SRD item or risk id can be cited.
- Owner decisions D-031 and D-032 [PLAN:3-4].
- Evidence: FINDINGS (tables 1 to 3, "The jump"). Related ticket: T-001 (AC4 and AC5 there give the Cartographer figures the spike compares against).
- Standards: CLAUDE s1 (evidence), s2 (motion), s3 (mocks, ROS-free logic), s5 (Pi resources).

## Open questions
Each has a recommended default, offered for the owner to accept or change. None is assumed in the criteria above.

1. Which failure must the camera fix: A (frame jumps at rest, AMCL spread), B (drift while driving, no encoders), or C (independent measurement of where the robot stopped for the 5 cm test)? Default: C and B for the spike (it can measure both from the same bag), A only if T-001 shows jumps persist.
2. Run T-001 first? Its result decides how much a camera must fix. Default: yes, run T-001 first (it needs no new tooling); the spike can still be recorded on the same pass if the owner prefers one trip.
3. What numbers define "camera tracks well"? Need: maximum mean and worst-case per-cross error (cm), maximum step at rest (cm), maximum allowed tracking-loss time. Default: leave blank; report numbers and decide with the owner after seeing them. A starting suggestion for discussion only: mean error under 5 cm, worst under 10 cm, no loss.
4. How many passes and how long at rest? Default: at least 3 Set P passes (matches T-001 AC4's n of 3 or more) plus a 10 minute rest recording (matches T-001 AC3), and 5 guarded 1 m drives for B (T-001 AC5).
5. Is the floor-tape position accurate enough to be the reference? Placement is "good to a few cm" [FINDINGS Table 3 note], which is the same order as the 5 cm target. Default: accept it for the spike, report it as a floor on the error, and ask whether a better reference (survey marks, a measured laser distance) is wanted for C.
6. What Pi cost is acceptable for recording (CPU cores, load average, temperature, disk and storage)? Known: Cartographer alone about 1.6 cores [CLAUDE s5]; load average 9 to 18 and 82.7 C with no fan, source `auto-mapping-run3.md:188` and T-001 Follow-ups. Default: record camera at low rate and size only, accept at most 0.25 cores and no rise in load average above the noise of the baseline; owner to confirm.
7. The 2026-09-28 claim (Cartographer moved off the Pi over Wi-Fi, pose ran away, scans about 90 ms late with jitter) is from the task brief; I could not find it in `MODULE.md`, `BUILD_LOG.md` or the docs. `BUILD_LOG.md:155` mentions only a 2026-09-25 runaway-on-late-scans fault. Is there a record to cite? Default: cite BUILD_LOG.md:155 and treat the 90 ms figure as unsourced. Update 2026-10-27: the owner's session memory note `tortoisebot-robot-run-recipe.md` records the 09-28 incident (scans about 90 ms late, jittery); it is not in the repo, so the figure stays owner-supplied until the note's content is added to a repo document.
8. Is the camera calibrated (intrinsics, extrinsics to base_link, exposure/timing against the IMU and lidar)? I found a tilt of 4.5 deg (4.75 deg measured mid-run 3) but no intrinsic calibration or time-sync record. Default: treat both as unknown and include them as things to establish in the spike report.
9. Is the camera's field of view and frame rate sufficient (the OV5647 on a robot looking 4.5 deg down at the floor sees floor and low walls)? Not stated in any source I read. Default: unknown; the spike result will show.
10. Is the Pi's camera pipeline free during Set P (the camera also feeds `/scan_fused`)? Whether a second consumer of the camera can run without changing the floor-scan load is not measured. Default: record the same stream, measure the cost, do not change the existing consumers.
11. Fiducial tags: if the camera does not track well, do you want tags considered now in requirements, or only after the spike? Default: only after the spike.
12. Where is the bag stored, and may a bag of this size be copied over Wi-Fi to the box at the owner's chosen time? Default: record to the Pi's local storage, copy by hand after the pass, confirm free space before recording.
13. Are the passes allowed in the room where the 2026-10-27 jump happened (so that jump-type events could occur on tape)? Default: yes, same room and set, so the comparison is like for like with FINDINGS.
14. Is "visual SLAM offline on the box" allowed to use the GPU? Default: ask at job time, one at a time, under the shared-machine rule [CLAUDE "Commands"].
