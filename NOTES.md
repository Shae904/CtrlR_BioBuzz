# NOTES.md: handoff for the CtrlR BIOBUZZ robot

A braindump for whoever works on this robot next, human or agent. Written 2026-10-22 by Ryan's Claude session, which
did most of the code since the Pedro 3 upgrade. Treat it as a coworker's notes: facts and history you can build on,
not rules. Keep it current: when you change something it describes, change it here too, and add a line to the log at
the bottom.

Everything below is in the state of CtrlR_BioBuzz `master` at `c605c7c` (PR #14) unless it says otherwise.

---

## 1. What's where

| Path | What it is |
|---|---|
| `vibecadded-ftc/CtrlR_BioBuzz` (this repo, github `Shae904/CtrlR_BioBuzz`) | the robot code. FTC SDK 12.0.0, Pedro Pathing 3 (revhub 3.0.2 + tuning 1.0.3-beta.1), Panels (fullpanels 1.0.18). Java 8 source level. |
| `vibecadded-ftc/biobuzz-sim` (github `inevident/ftc-robot-sim`, private) | a desktop simulator that runs this TeamCode **unchanged** against a shim of the FTC SDK. Field + robot physics from the official CAD. Every auto and the shot formula were developed there first. See its README. |
| `vibecadded-ftc/shooter-model` | the Python model behind the flywheel-speed formula (`DERIVATION.md`, `biobuzz_model.py`, `fit_formula.py`). |
| `vibecadded-ftc/fields/biobuzz-2026-27` | official field STEP, the manual (`manual.txt`), AprilTag library. |
| `vibecadded-ftc/ROBOREBELSDECODE` | last season's (DECODE) code, branch `v3-worlds`. This repo is a fork of it. The sim can run it too. |
| Fusion (Autodesk) | the CAD. Team `roborebels/biobuzz`, design **"Worlds Bot" v13** = this robot; the turret is the linked `ghettoturret` (v10 is the latest, with the 35° Limelight mount). The hood is parametric, see §3. |
| Panels | `http://192.168.43.1:8001` on the robot Wi-Fi: live `@Configurable` fields and the Graph. |
| Pedro AutoTune | `http://192.168.43.1:10158` (page) + websocket on port 12649. |
| Limelight web UI | `http://192.168.43.1:5801`. |

Workflow that has worked: branch → PR to `master` → merge (`gh pr merge --merge`). No attribution lines in commits. Ryan
wants robot changes pushed right away; the simulator gets updated afterwards, and only when Ryan says the sim can be
worked on.

## 2. The robot (CAD facts, mm unless stated)

Frame conventions used everywhere in the sim and in these notes: origin at the robot centre (geometric centre of the
four wheel contact patches) on the floor, **x forward (toward the intake), y left, z up**. Pedro's field frame is
inches, origin at the red/audience corner, x toward blue, y toward the rear wall, heading CCW from +x.

**Drivetrain** `worlds drivetrain:1`
- 4 × goBILDA 104 mm mecanum (3625-xxxx-0104), 4 × Yellow Jacket 435 RPM (5203-2402-0014, 13.7:1, 384.5 PPR).
- Track 387.5 (15.26 in), wheelbase 263.9 (10.39 in). Footprint from centre: +250.8 forward (the intake wheels), −186.9 back, +212.65 left, −242.15 right (the turret's input assembly). 455 × 438 mm, 451 mm tall.
- Motors `fl`/`bl` REVERSE, `fr`/`br` FORWARD (Robot.java). Pedro-tuned: xVelocity 80.21 in/s, yVelocity 65.97 in/s, zero-power decel −57.3 / −64.6 in/s².

**Odometry** (changed 2026-10-21, PR #14)
- goBILDA **Pinpoint** with **two** pods: the **left** parallel pod on the Pinpoint's **X** port, the **strafe** pod on **Y**. The right pod is no longer read. Pods: goBILDA 4-bar, 32 mm wheel, 2000 CPR.
- **Tuned on the robot 2026-10-22** (CommanderSleepy, Pinpoint Tuner): `xPodOffset 7.48184`, `yPodOffset 2.46908`, both pod directions REVERSED. The X offset matches the CAD's 7.5 in. The Y offset came out +2.47 where the CAD puts the strafe pod 2 in *behind* centre (−2.0): either the pod moved or the tuner's spin step read the sign the other way; a quick check is the Pose Test while spinning the robot 360° by hand: sideways drift > 1 in means the Y offset sign is wrong.
- Device name in the robot configuration must be exactly `pinpoint`, on a Control Hub I2C port (1–3; port 0 is the built-in IMU's). As of 10-22 the Pinpoint is confirmed connected and reporting data over ADB.

**Turret** `ghettoturret:1`
- Axis at robot (−13.8, −46.25) mm, i.e. 13.8 behind and 46.25 right of centre; turret plane 247.9 mm above the floor.
- Motor: Yellow Jacket 435 RPM, 2.5:1 spur (40 T) → `Outtake.ticksPerDeg = 2.6701388889` (384.5 × 2.5 / 360). Motor `turret` REVERSE.
- Code convention: turret position = encoder degrees + `AllianceColor.getSteadyState()` (RED 90, BLUE 270). **180 = shooter forward, 90 = shooter pointing left, 270 = right.** So at INIT the shooter must point **left on RED, right on BLUE**; the encoder zeroes there.
- Rotating inertia ≈ 0.03 kg·m² (sim estimate); the code's aim is a PID on Limelight tx (`Limelight.AIM_*`).

**Shooter** (flywheel + hood)
- 2 × Yellow Jacket 6000 RPM (5203-2402-0001) on a 1:1 belt, 28 CPR → "ticks/s" in telemetry is encoder counts/s (1140 ticks/s ≈ 2440 RPM). Full power ≈ 2300 ticks/s (`kV = 0.0004348`). `flywheel1` FORWARD, `flywheel2` REVERSE; velocity is read from `flywheel1`. Rotor inertia 6.06e-4 kg·m² (Fusion mass properties).
- Hood (`BlankHood v2`, parametric user params in `ghettoturret`): BallDia 71.12, BallCompression 3, TireSeatOffset 7.5, TireSeatRadius 46.48, HoodRadius 113.8, HoodCupDia 80, HoodEntryRadius 15, **HoodExitAngle 48°**. The ball drops into the gap between the two tires and rides each tire's inner shoulder (not the 96 mm OD). Hood exit tangent 48°; launch point 4.8 mm ahead of the turret axis, **378 mm above the floor**.
- **Measured launch angle: ~58° (Logger Pro, 2026-10-21).** The code currently assumes 52.6° (see §4). Ryan asked for the 58° version to be worked out but NOT implemented yet.

**Limelight 3A** (on the turret, `LIMELIGHT3ACAD_STEP (2):3`)
- Lens 64.4 mm behind the turret axis, 81.5 mm right of the shot line, **441 mm above the floor, pitched 35° up**, yaw 0 (looks along the shot). Housing is mounted inverted relative to the STEP's native orientation.
- Pipelines: **1 = RED** (tags 30–37), **2 = BLUE** (38–45), AprilTag 36h11, **marker size 82.55 mm** (3.25 in), Full 3D on, 640×480. Set up on 10-19; detector downscale should be 1 and quality threshold ≈1.5 if tags flicker. **Click "Stop Ignoring NetworkTables Index"** in the LL web UI or the robot's pipeline switch is ignored.

**Intake / hopper**
- Front roller of 9 compliant wheels (Ø48), axis 217 mm ahead of centre, 90 mm above the floor, 306 mm wide. Only POLLEN (2.8 in) fits the 80 mm channel; NECTAR (3.6 in) does not. Hopper holds 4 (the preload count). Motor `intake` REVERSE, `POWER_INTAKE = 1`.
- Gate ("slapper", added to the robot 2026-10-22): servo `flapper`, driven by `Intake.setFlapper(boolean up)`. **Starts DOWN** (shut: the hopper holds balls while intaking) at construction and at INIT; **UP** (open: balls feed into the flywheel) only while triangle is held in teleop, or while an auto shoots. Positions `Intake.FLAPPER_UP 0.1` / `FLAPPER_DOWN 0.35` are still DECODE's numbers: if the slapper moves the wrong way, swap them (Panels-live, then copy into the code). The DECODE `cycler` servo and its cycle state machine are gone (PR #16); `Intake` is just the roller motor + the flapper.
- Other servos: `tiltA`/`tiltB` (folded .98/.08, deployed .49/.55), `rgb` (indicator: .5 locked on, .277 not aimed, .611 no target).

**Hardware map names** (all of them): `fl fr bl br flywheel1 flywheel2 turret intake` (motors), `flapper tiltA tiltB rgb` (servos), `imu`, `limelight`, `pinpoint`.

## 3. The field (what the code and autos assume)

- Interior 141.05 in (CAD), origin red/audience corner. 180° rotationally symmetric (not mirrored): BLUE poses = RED's rotated about the centre (`FIELD_IN − x, FIELD_IN − y, heading + 180°`). The autos do this in `BaseHiveAuto.fieldPose`.
- RED hive pivot at (57.78, 70.52) in, 43.95 in up; cells "RED AUDIENCE" (tags 34–37, up at start, opening centre (57.8, 54.8) in) and "RED SCORING" (30–33, opening (57.8, 86.25) after the first tip). BLUE mirrored at x = 83.28, "BLUE SCORING" up at start (42–45), "BLUE AUDIENCE" (38–41).
- Cell opening centre **1.509 m** high, tilted 30° toward the side it faces; the tags are 73 mm **beyond** the opening (`OPENING_BEYOND_TAGS_M = −0.073`), tag cluster 1.261 m up when the cell is up, 0.905 m when down. Tags face the same side the opening faces, down and outward.
- Hive tips at **186.5 g** in the up cell (official setup guide: 8 pollen or 3 nectar + 3 pollen tip; 7 pollen / 3N+2P don't). Cells start with 3 nectar, so the **first tip needs 3 pollen, later ones 8**. Pollen 24.9 g, nectar 41.3 g. A tip takes ~1.2 s and spills the cell.
- Red loading zone x 0–11.4, y 94.4–117.1 in (PARK = partly inside it); red garden along the audience wall x 0–23.1, y 0–2.4 with 4 pollen at x 1.4/4.2/7.0/9.8. Flowers: columns at (47.13, 138.57) rear-red, (2.48, 47.13) red wall, mirrored for blue; 4 pollen each, retrievable from the bottom opening.
- Scoring: LEAVE 3, PARK 5 (auto and teleop), TIP 20, element left in up cell 2 (teleop only), garden 1, flower 2 + bottom nectar 5.

## 4. The shot formula (history matters here)

`Limelight.calculateTargetVelocity()`: `ticks = d·(SHOT_A·d − SHOT_B)^−0.5 / SHOT_K + SHOT_OFFSET − vParallel·SHOT_MOVE_GAIN`, with `d` = camera→opening horizontal distance in m (`tagDistance + OPENING_BEYOND_TAGS_M`), rounded to 20. `TOF_A/B` give time of flight for leading moving shots.

Current constants (PR #13, 2026-10-19): `SHOT_A 0.280894269484, SHOT_B 0.240816326531, SHOT_K 0.0025098411518, SHOT_OFFSET −261, TOF_A 1.505, TOF_B 1.44`. Model behind them: 52.6° launch (48° hood + DECODE's 4.6° spring-back), **no slip loss** (eta 1.013), 2.80 in pollen, fitted so **1140 ticks/s scores from the auto corner** (camera 1.54 m from the opening), which the team measured on the robot. Predicts 1146 at 2 m, 1205 at 2.5 m, 1280 at 3 m.

Why it moved: the 10-16 constants (eta 0.859, the DECODE robot's 14% slip) asked for 1429 at that corner, and 1140 can't reach the opening at any angle with that loss. One shot fixes eta × angle together; a Logger Pro video since says the launch is **~58°**, not 52.6°. The 58° refit (NOT applied) would be `SHOT_A 0.295833167927, SHOT_B 0.212244897959, SHOT_K 0.0021473714554, SHOT_OFFSET −314, TOF_A 1.870, TOF_B 1.54`, `SHOT_MOVE_GAIN 93·cos(58°)`; it gives the same 1140 at the corner but +53 at 2 m and +106 at 3 m. A shot from ~3 m with the current code decides: lands short → 58° is right.

Manual presets `Outtake.medSpeed 1380 / farSpeed 1560` are still the OLD model's values; with the 52.6° model they'd be ~1146 / 1311.

Flywheel control: `power = (kS + kV·target + kP·error) · 12 / Vbatt` (`Outtake`, voltage-compensated). `kS 0, kV 0.0004348, kP 0.003`; the team tuned kS/kV/kP in Panels on 10-19 ("to our liking"), **check that the values in `Outtake.java` are the tuned ones**, Panels edits don't persist. `FLYWHEEL PID TESTING` opmode: triangle = kS/kV sweep, square = step test, circle = hold + shot recovery.

Aim: `AllianceColor.getAimPose()` is the tx (deg) the turret centres the tags at (RED −5, BLUE −1). The autos use their own per-spot aim points (`AIM_TX_*`, −4.5 and −1.0), tuned in the sim: the camera sits 82 mm beside the shot line, so the aim point depends on where you shoot from. In the sim, BLUE's −1 missed; expect to retune BLUE.

## 5. Pedro Pathing 3 and AutoTune

- `pedroPathing/Constants.java`: `drivetrainConfig` (MecanumConfig), `localizerConfig` (PinpointConfig, §2), `foresightConfig` (ForesightConfig: velocities/decels carried from 2.x tuning, brake/coast models marked ESTIMATE). `createFollower(hardwareMap)` builds `new Follower(localizer, drivetrain, new Foresight(config))`; `Robot` calls it.
- `pedroPathing/Tuning.java`: the `@Tuner` entry points (Mecanum, **Pinpoint**, Foresight, Tests). Procedures in `pedroPathing/procedures/` are copies of the Pedro quickstart's (github `Pedro-Pathing/Quickstart`, `TeamCode/.../pedro/procedures`); `PinpointTuner` is the quickstart's verbatim. The old `ThreeWheelTuner` was removed.
- Pinpoint Tuner steps: device name + pod type → push forward (X direction) → push left (Y direction) → spin 180° CCW in place (offsets) → prints a `PinpointConfig` block.
- **After the localizer change, re-run the Foresight Tuner**: its numbers were measured through the old localizer.
- Autos run **MANUAL bulk caching** with `clearBulkCache()` each loop (like BaseTeleop). `Robot` sets AUTO; with the old 3-wheel localizer that read one pod a loop late and drifted 4–5 in over the 2 TIP route in the sim.

### AutoTune "tuner disconnected" (2026-10-22, resolved)
Every tuner's websocket dropped ~1 s after clicking. The mechanism: the page sends `init`, the server creates the procedure and, if anything in it throws, closes the socket with the exception's message (also in logcat, thread `Pedro-Tuning`, tag `WebSocketServer`). The throw was the `pinpoint` device missing from the active robot configuration; once it was configured (Control Hub I2C port, type "goBILDA Pinpoint Odometry Computer", name `pinpoint`) the Pinpoint and Foresight tuners ran and produced the blocks now in `Constants`. If it ever comes back: read that console line / logcat first, it names the exception.

## 6. Autonomous (in `Auton/`, written in the sim, 48 and 20 points there)

- `BaseHiveAuto`: shared toolkit. Alliance poses (`fieldPose`), `drive(path, timeout)` + `driveTo` (a short settle path if Pedro stopped >1 in off), Limelight aim with a per-spot aim point (`setAimPoint`), the turret resting toward where the cell will be while driving (tracking a cell behind the robot by odometry ran it into its ±180° limit), `aimUntilReady` (2° and flywheel within 30 ticks/s, held 80 ms), `shoot(n)` one ball per intake pulse, `shootVolley()`, `stopAiming()` before parking. All `@Configurable`.
- **HIVE 4** (`Hive4/`): start RED (46, 7.36, 90°) / BLUE (95.05, 133.69, 270°) against the audience/rear wall; out to (46, 22), shooting spot (16.5, 16.5) 1.43 m from the up cell, 47° off its axis; volley of 4. The 3rd pollen tips (3 nectar already in the cell), the 4th spills. 20 + LEAVE.
- **2 TIP** (`TwoTip/`): same start; shoot 3 → tip; sweep the garden along the audience wall pressed on it (turn at (28, 15), slide to (25, 8.4), sweep to (12.3, 7.0) at 25 in/s) for 3 pollen; curve via (26, 40), (26, 95) to (15.5, 125.9) behind the hive, aim point −1°, shoot 4 into the cell that came up; rear flower (47.5, 117) → (47.5, 126.2) facing it, intake 1 s for its 4 pollen; back to (15.5, 125.9), shoot 4 → second tip (needs 8); park at (17, 106, 90°). 48 points in the sim, both alliances, lockstep and real time.
- Spin-up target while driving `SPINUP_TICKS = 1150`. The autos open/close the gate through `Intake.setFlapper`.
- **None of this has run on the real robot yet.** Things to expect to tune: aim points, feed timing (`FEED_FIRST_MS 150`, `FEED_INTERVAL_MS 233`, `FEED_MARGIN_MS 110`), whether the real intake grabs pollen pressed against the wall and out of the flower, Pedro's following accuracy (then the settle thresholds).
- Shooting geometry limits (from the sim): from closer than ~1.2 m the ball can't climb to the opening (needs ≥0.96 m horizontal at any speed); comfortable 1.3–1.45 m; the speed formula has a singularity below ~1.3 m (`SHOT_A·d − SHOT_B` → 0). From the spilled-pollen pile right under the hive you cannot score: back off to ~1.4 m first.

## 7. Teleop (`TeleOp/BaseTeleop`, RED/BLUE)

Field-centric drive on gamepad1 left stick, turn right stick (either pad). Left bumper (gp1) resets yaw. **RB (either pad) = auto-aim + auto flywheel speed** from the Limelight. Triangle = flapper open (SHOOT). dpad down (gp1) stops the flywheel; gp2 right/left stick buttons toggle auto speed; with auto off, gp2 dpad up/down/right = far/med/cycle presets, gp2 LB = one formula read. Triggers = intake in/out. gp1 dpad up/left = tilt deployed/folded. Telemetry goes to both the DS and Panels (`Robot` wraps `opMode.telemetry` in Panels' `JoinedTelemetry`, PR #12) so Panels' Graph can plot `Vel Current`, `Vel Target` etc. The Graph plots nothing until you pick series ("All"), one shared y-axis, 75 ms updates.

## 8. Pitfalls we've hit (so you don't)

- Panels `@Configurable` edits don't survive a restart; copy tuned values into the code.
- SDK merges overwrite `build.dependencies.gradle`: re-add the bylazar + dairy repos, revhub, tuning and fullpanels after any SDK update. Panels must be ≥1.0.15 on SDK 12. Sloth 0.3.2 comes from the Dairy repo and is required (Panels configurables/opmodecontrol and Pedro tuning need it).
- `tuning:1.0.2` crashes Sloth at robot start (TunerScanner NPE); `1.0.3-beta.1` is what we run (the 10-22 "disconnected" scare turned out to be the Pinpoint missing from the robot configuration, not the library; the tuners worked once it was configured).
- Pedro 3 `ConfigVar<Double>` must be set with a double literal (`-2.0`, not `-2`).
- The Limelight web UI "ignores NetworkTables index" while you have a pipeline selected there: click the button or the code can't switch pipelines.
- Logger Pro angle: the camera must be perpendicular to the shot plane or the angle reads steep.
- The Pinpoint's IMU calibrates for ~0.25–0.5 s after INIT/reset: keep the robot still.
- Both hives' tags are only visible from the side the up cell faces (down + outward); from behind a hive you see nothing. The autos' shooting headings are chosen so the cell is in the turret's rest direction on arrival.

## 9. Simulator, briefly (`biobuzz-sim`)

Run `./gradlew :sim-app:run` → `http://localhost:7070` (Java 17+; the Android Studio JBR works: `JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"`). It compiles `../CtrlR_BioBuzz/TeamCode` as-is, so the DS panel there lists these exact opmodes, with "place at start pose" read from the auto's own `setPose`. `./gradlew check` = ~580 tests (shot validation, Pedro localization round trip, both autos end to end, hive tip, field rules). Docs: `docs/PHYSICS.md` (every model + CALIBRATE list), `docs/research/07-robot-cad-measurements.md` (the CAD numbers above), `08` (field CAD), `09` (hive tip spec).

**Known gap:** the sim has no `GoBildaPinpointDriver` stand-in yet, so since PR #14 the team's `Constants.createFollower` cannot run in the sim (localization + auto tests will fail there) until a Pinpoint model is added. Work-in-progress files for that are parked outside the repo; Ryan wants sim work to wait until he says so.

## 10. People / accounts

Ryan Lim (github `inevident`, commits as `inevident <ryanlimpre@gmail.com>`), teammate `shaizebos` (moved the autos into `Auton/Hive4` and `Auton/TwoTip` on 10-19). Repo owner `Shae904`.

---

## Log (newest first; add a line when you change the robot)

- 2026-10-22: PR #16 removed the DECODE cycler servo and the cycle state machine (`updateAutoCycle`, `FlapperState`, `CYCLER_LOCKED`); `Intake` = roller + flapper. (Claude/Ryan)
- 2026-10-22: Flapper logic reworked (PR #15): `Intake.setFlapper(up)`, starts DOWN at construction and INIT, UP only while triangle is held, autos use it; `cycler` optional. Positions still DECODE's 0.1 / 0.35, to be checked on the slapper. (Claude/Ryan)
- 2026-10-22: CommanderSleepy: Pinpoint tuned (REVERSED/REVERSED, 7.48 / +2.47 in), Foresight re-tuned (full block), flapper servo re-enabled in Intake with hold-to-SHOOT in teleop. Not logged here at the time; recorded by Claude on review.
- 2026-10-22: AutoTune websocket drops ~1 s after any tuner starts (all four). Pinpoint confirmed connected and reading via ADB. Suspect tuning 1.0.3-beta.1; propose pinning tuning 1.0.1. Not yet changed. (Claude/Ryan)
- 2026-10-21: PR #14 Pinpoint odometry (left + strafe pods), Pedro PinpointLocalizer + quickstart PinpointTuner; ThreeWheelTuner removed. Logger Pro measured the launch angle ≈58°; 58° refit computed but not applied. (Claude/Ryan)
- 2026-10-19: PR #13 shot formula recalibrated to 1140 ticks/s from the auto corner (no slip loss, 52.6°). PR #12 telemetry to Panels. PR #11 HIVE 4 + 2 TIP autos (from the sim); shaizebos moved them into subpackages. Limelight pipelines set up for BIOBUZZ tags. Flywheel kS/kV/kP tuned in Panels. (Claude/Ryan/shaizebos)
- 2026-10-16: PR #7–#10: Pedro 3 migration, tuning pinned to 1.0.3-beta.1, shot formula refit to ghettoturret v10 + corrected cell geometry, FLYWHEEL PID TESTING tools. (Claude/Ryan)
- 2026-10-15: SDK 12 + Panels/Sloth deps (PR #5/#6), Limelight multi-tag hive detection, 35° camera, CAD-derived shot formula.
