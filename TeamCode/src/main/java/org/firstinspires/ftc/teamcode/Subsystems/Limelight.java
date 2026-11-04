package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AllianceColor;

@Configurable
public class Limelight {

  public static double AIM_Kp = 0.02;
  public static double AIM_Ki = 0;
  public static double AIM_Kd = 0.002;
  public static double AIM_Ks = 0.04;

  public static double AIM_DEADBAND = .3;
  /** Turret power per deg/s of the target's own motion (the robot turning), feed-forward. ~1/free speed (~1040 deg/s). */
  public static double AIM_Kv = 0.0008;

  /** "Aimed" light: |turret error| under this fraction of the opening's window from here, and never over the cap. */
  public static double AIM_WINDOW_FRACTION = 0.5;
  public static double AIM_RGB_THRESHOLD = 2;
  /** A Limelight result older than this (ms) is no target (camera stopped sending). */
  public static int STALE_MS = 300;
  /** Flywheel target before the first tag reading (ticks/s): the auto-corner shot (1.54 m), the formula's ~1140. */
  public static double DEFAULT_SHOT_TICKS = 1150;
  /** Teleop pre-aim from odometry: back to the rest direction if no tags are found within this (ms); re-press to retry. */
  public static int ODOMETRY_PREAIM_TIMEOUT_MS = 1500;
  /** The opening of each RED CELL when it is up (in, Pedro): the teleop pre-aim from odometry; BLUE is rotated. */
  public static double[] OPENING_RED_AUDIENCE_IN = {57.8, 54.8};
  public static double[] OPENING_RED_SCORING_IN = {57.8, 86.25};

  // --- BIOBUZZ AprilTags (manual 9.9 / Figure 9-17): 16 tags, 4 per CELL, on each CELL's underside ---
  //   red scoring 30-33, red audience 34-37, blue audience 38-41, blue scoring 42-45
  // Heights/offsets come from the official field CAD (floor = top of tiles).
  public static double LENS_HEIGHT_M = 0.441;          // Limelight lens above floor (ghettoturret v10 CAD, 35 deg mount)
  public static double CAM_PITCH_DEG = 35.0;           // Limelight lens axis above horizontal (ghettoturret v10 CAD)
  public static double UP_CELL_TAG_HEIGHT_M = 1.261;   // tag cluster height when its CELL faces up (the scoring target)
  public static double DOWN_CELL_TAG_HEIGHT_M = 0.905; // tag cluster height when its CELL faces down
  // The CELL opening vs its tags and the camera vs the shot line: AimGeometry (the aim and the formula's distance).
  /** Aim trim (deg, + = right) on top of the geometry: one number for a camera yaw error or a sideways throw. */
  public static double AIM_TRIM_DEG = 0;

  // --- Shot formula from the CAD + physics model (vibecadded-ftc/shooter-model/DERIVATION.md) ---
  // ticks/s = d * (SHOT_A * d - SHOT_B)^-0.5 / SHOT_K + SHOT_OFFSET, d = horizontal camera -> CELL opening (m)
  // Calibrated 2026-10-19 on the robot: 1140 ticks/s scores from the auto corner (camera -> opening 1.54 m), where
  // the 10-16 fit asked for 1429. The 10-16 fit carried over the DECODE robot's 14% slip loss (eta 0.859) and a small
  // spring-back (49.8 deg); 1140 is only possible with no slip loss (at 0.859 the ball can't reach the opening at any
  // launch angle). Model now: no slip loss (eta 1.01, the ball leaves at ~50% of the tire speed), launch 52.6 deg (the
  // 48 deg hood + 4.6 deg spring-back, as measured on DECODE), launch point 378 mm, 2.80 in POLLEN, opening centre
  // 1.509 m. Valid ~1.4-3.6 m: 1.54 m -> 1140, 2.0 m -> 1146, 2.5 m -> 1205, 3.0 m -> 1280. Still to confirm with a
  // shot from ~3 m (if that one lands high, the launch is flatter: refit with the measured speed).
  public static double SHOT_A = 0.280894269484;
  public static double SHOT_B = 0.240816326531;
  public static double SHOT_K = 0.0025098411518;
  public static double SHOT_OFFSET = -261;
  public static double SHOT_MOVE_GAIN = 93 * Math.cos(Math.toRadians(50)); // ticks per m/s toward the goal (DECODE tuning)
  // POLLEN time of flight to the CELL opening: sqrt(2 * (TOF_A * d - TOF_B) / 9.8) seconds (same model, 2026-10-19)
  public static double TOF_A = 1.505;
  public static double TOF_B = 1.44;

  /**
   * The same fit for a 58 deg launch (Logger Pro measured ~58 deg on 2026-10-21; ghettoturret v11 has HoodExitAngle
   * 53 deg): both give 1140 at the auto corner (1.54 m), the 58 deg one less closer in (1.3 m: 1140 vs 1200) and MORE
   * farther out (2.0 m: 1200 vs 1140, 2.5 m: 1280 vs 1200). Off: on the robot the shots go over at MEDIUM range after a
   * full spin-up, where this fit asks for even more (2026-11-04). Panels switch, for experiments only.
   */
  public static boolean SHOT_MODEL_58_DEG = false;
  public static double SHOT58_A = 0.295833167927;
  public static double SHOT58_B = 0.212244897959;
  public static double SHOT58_K = 0.0021473714554;
  public static double SHOT58_OFFSET = -314;
  public static double SHOT58_MOVE_GAIN = 93 * Math.cos(Math.toRadians(58));
  public static double TOF58_A = 1.870;
  public static double TOF58_B = 1.54;
  /** Flywheel trim added to the formula (ticks/s, + = faster): Panels, or gamepad2 dpad up/down in the teleop. */
  public static double SHOT_TRIM_TICKS = 0;
  /** Closer than this (m, camera to opening) the formula's speed climbs steeply (52.6 deg: 1.2 m -> 1279): warn. */
  public static double TOO_CLOSE_M = 1.3;
  /** The 3D distance may differ from the tag-height distance by this fraction before telemetry warns (marker size?). */
  public static double DISTANCE_CHECK_TOLERANCE = 0.15;


  // --- Per-CELL pipelines (Limelight web UI): each sees one CELL's 4 tags. The TELEOP picks the CELL from odometry
  //     (HiveTargeting, targetCell); the autos stay on the alliance pipeline (both CELLS, getLLPipelineTeleOP). ---
  public static int PIPELINE_RED_SCORING = 3;    // tags 30-33: the red CELL facing the far wall
  public static int PIPELINE_BLUE_SCORING = 4;   // tags 42-45: the blue CELL facing the far wall
  public static int PIPELINE_RED_AUDIENCE = 5;   // tags 34-37: the red CELL facing the audience wall
  public static int PIPELINE_BLUE_AUDIENCE = 6;  // tags 38-41: the blue CELL facing the audience wall
  /** A failed pipeline switch is retried after this long (ms). */
  public static int PIPELINE_RETRY_MS = 250;

  public final Limelight3A limelight;
  private final AllianceColor currentColor;
  /** The CELL whose tags count, or null for either CELL of our HIVE (the autos). */
  private HiveTargeting.Cell targetCell;
  private int requestedPipeline;
  private int switchedPipeline = -1;
  private final ElapsedTime pipelineRetry = new ElapsedTime();
  private final ElapsedTime aimTimer = new ElapsedTime();

  public final Servo rgb;

  private LLResult currentGoal;

  private double lastCalculatedVel = DEFAULT_SHOT_TICKS;
  private double aimIntegral = 0;
  private double aimLastError = 0;

  private double vPerpendicular;
  private double vParallel;

  // Result of the multi-tag hive detection (see updateHiveTarget)
  private boolean hiveSeen = false;
  private double aimCorrection = 0;  // turret turn that puts the shot line through the opening (deg, + = right)
  public double tagDistance;         // horizontal camera -> tag cluster centre (m)
  public int hiveTagCount;           // how many of our hive's tags were used
  private double cameraYawRad;       // the camera's (= shot line's) field heading: robot heading + turret angle
  private final AimGeometry.Cluster[] clusters = {
      new AimGeometry.Cluster(), new AimGeometry.Cluster(), new AimGeometry.Cluster(), new AimGeometry.Cluster()};
  private double windowDeg = Double.NaN;  // the opening's window from here (deg), for the "aimed" light

  // Field-held aim (TurretTracker): camera delay, robot turning, lost tags
  private final TurretTracker tracker = new TurretTracker();
  private double headingDeg;             // robot heading now (deg, CCW)
  private double robotXIn;
  private double robotYIn;
  private double yawRateDegS;            // robot turn rate (deg/s, CCW)
  private double lastFrameTimestamp = Double.NaN;
  private long lastAimCallNs;
  /** Where the turret target came from: REST, VISION, HOLD (tags just lost), ODOMETRY (teleop pre-aim), LAST (autos). */
  private String aimSource = "REST";
  private boolean wasAiming;
  private long odometryPreaimSinceNs = -1;   // when this aiming session's odometry pre-aim started (-1: not yet)
  /** Our tags seen this frame without a 3D pose (ignored: up/down can't be checked); > 0 means Full 3D is off. */
  public int tagsWithout3d;
  /**
   * Cross-check of the target CELL's distance: (distance from the tags' known up-cell height and the camera's pitch) /
   * (distance from the Limelight's 3D poses). ~1 when both agree; ~2 or ~0.5 = the pipeline's marker size is wrong
   * (it scales the 3D distance), NaN = no target.
   */
  public double distanceCheckRatio = Double.NaN;

  public double target;

  public double distance;
  public double error;

  public double turretPosition;
  private double lastTarget;

  public Limelight(LinearOpMode opMode, AllianceColor color) { // Constructor
    HardwareMap hardwareMap = opMode.hardwareMap;
    this.currentColor = color;
    this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    this.limelight.start();
    // Both CELLS of our HIVE; the TELEOP then narrows it to one CELL (targetCell)
    this.requestedPipeline = this.currentColor.getLLPipelineTeleOP();
    if (this.limelight.pipelineSwitch(this.requestedPipeline)) {
      this.switchedPipeline = this.requestedPipeline;
    }

    this.lastTarget = this.currentColor.getSteadyState();
    this.turretPosition = this.currentColor.getSteadyState();

    this.rgb = hardwareMap.servo.get("rgb");
    this.rgb.setPosition(.5);
  }

  public void setTarget(double pos) {
    this.lastTarget = pos;
  }


  /** The pipeline that sees only {@code cell} of {@code color}'s HIVE. */
  public static int pipelineFor(AllianceColor color, HiveTargeting.Cell cell) {
    if (color == AllianceColor.RED) {
      return cell == HiveTargeting.Cell.SCORING ? PIPELINE_RED_SCORING : PIPELINE_RED_AUDIENCE;
    }
    return cell == HiveTargeting.Cell.SCORING ? PIPELINE_BLUE_SCORING : PIPELINE_BLUE_AUDIENCE;
  }

  /** The CELL's index (tag id - 30) / 4: 0 red scoring, 1 red audience, 2 blue audience, 3 blue scoring. */
  static int cellIndex(AllianceColor color, HiveTargeting.Cell cell) {
    if (color == AllianceColor.RED) {
      return cell == HiveTargeting.Cell.SCORING ? 0 : 1;
    }
    return cell == HiveTargeting.Cell.AUDIENCE ? 2 : 3;
  }

  /**
   * Looks at one CELL of our HIVE only: switches the Limelight to that CELL's pipeline (once, retried if the switch
   * failed) and ignores every other tag, so the frames from the old pipeline right after a switch, or a Limelight
   * that ignores the switch, cannot aim at the other CELL.
   */
  public void targetCell(HiveTargeting.Cell cell) {
    if (cell != this.targetCell) {
      tracker.clearFix();                 // the held direction was the other CELL's
      lastFrameTimestamp = Double.NaN;
    }
    this.targetCell = cell;
    int pipeline = pipelineFor(currentColor, cell);
    if (pipeline == switchedPipeline) {
      requestedPipeline = pipeline;       // (a failed switch to another pipeline may have been asked for meanwhile)
      return;
    }
    if (pipeline != requestedPipeline || pipelineRetry.milliseconds() >= PIPELINE_RETRY_MS) {
      requestedPipeline = pipeline;
      pipelineRetry.reset();
      if (limelight.pipelineSwitch(pipeline)) {
        switchedPipeline = pipeline;
      }
    }
  }

  /** The pipeline the code asked for. */
  public int requestedPipeline() {
    return requestedPipeline;
  }

  /** The pipeline the Limelight's latest result came from, or -1 before the first result. */
  public int runningPipeline() {
    LLResult result = limelight.getLatestResult();
    return result == null ? -1 : result.getPipelineIndex();
  }

  /** True if the tag belongs to our alliance's HIVE (both of its CELLS). */
  private static boolean isOurHiveTag(int id, AllianceColor color) {
    return color == AllianceColor.RED ? (id >= 30 && id <= 37) : (id >= 38 && id <= 45);
  }

  /**
   * Uses every visible tag of our HIVE (not just the Limelight's primary target): groups them by CELL, picks the
   * upward-facing CELL (the higher cluster) and aims from the measured tag positions (AimGeometry): each tag's
   * horizontal bearing and range, its place in the row, the opening in front of the cluster, the camera beside the
   * shot line. Angles are taken without the Limelight crosshair, so a crosshair set in a pipeline changes nothing.
   * Only tags with a 3D pose count: the pose's height tells the up CELL from the down one (no 3D: tagsWithout3d).
   */
  private void updateHiveTarget() {
    this.hiveSeen = false;
    this.hiveTagCount = 0;
    this.tagsWithout3d = 0;
    this.distanceCheckRatio = Double.NaN;
    if (currentGoal == null || !currentGoal.isValid()) {
      return;
    }
    double pitch = Math.toRadians(CAM_PITCH_DEG);
    for (AimGeometry.Cluster cluster : clusters) {
      cluster.clear();
    }
    double[] sumHeight = new double[4];
    double[] sumRange3d = new double[4];
    double[] sumRangeFromHeight = new double[4];
    int[] count = new int[4];
    int onlyCell = targetCell == null ? -1 : cellIndex(currentColor, targetCell);
    for (FiducialResult tag : currentGoal.getFiducialResults()) {
      int id = tag.getFiducialId();
      if (!isOurHiveTag(id, currentColor)) {
        continue;
      }
      int cell = (id - 30) / 4; // 0: red scoring, 1: red audience, 2: blue audience, 3: blue scoring
      if (onlyCell >= 0 && cell != onlyCell) {
        continue; // not the CELL the TELEOP targets
      }
      // Ray to the tag, rotated from the tilted camera into a level frame
      // (camera space: +x right, +y down, +z out of the lens; tx/ty are pinhole angles from the optical axis)
      double tanTx = Math.tan(Math.toRadians(tag.getTargetXDegreesNoCrosshair()));
      double tanTy = Math.tan(Math.toRadians(tag.getTargetYDegreesNoCrosshair()));
      double rayFwd = Math.cos(pitch) - tanTy * Math.sin(pitch);
      double rayUp = Math.sin(pitch) + tanTy * Math.cos(pitch);
      // Horizontal bearing in the level frame (+ = right)
      double bearing = Math.atan2(tanTx, rayFwd);
      Pose3D pose = tag.getTargetPoseCameraSpace();
      Position p = pose == null ? null : pose.getPosition().toUnit(DistanceUnit.METER);
      if (p == null || p.z <= 0.05) {
        tagsWithout3d++;                  // no 3D pose: can't tell an up CELL from a down one, so not used
        continue;
      }
      double fwd = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
      double up = p.z * Math.sin(pitch) - p.y * Math.cos(pitch);
      sumHeight[cell] += LENS_HEIGHT_M + up;
      double range = Math.hypot(p.x, fwd);
      clusters[cell].add((id - 30) % 4, bearing, range);
      sumRange3d[cell] += range;
      // the same distance from the ray's elevation and the up CELL's tag height (independent of the marker size)
      sumRangeFromHeight[cell] += rayUp > 0.01
          ? (UP_CELL_TAG_HEIGHT_M - LENS_HEIGHT_M) * Math.hypot(tanTx, rayFwd) / rayUp : range;
      count[cell]++;
    }

    // Upward-facing CELL = the higher cluster, measured from the 3D poses (a down CELL is never a target)
    double midHeight = (UP_CELL_TAG_HEIGHT_M + DOWN_CELL_TAG_HEIGHT_M) / 2.0;
    int best = -1;
    double bestHeight = Double.NEGATIVE_INFINITY;
    for (int c = 0; c < 4; c++) {
      if (count[c] == 0) {
        continue;
      }
      double height = sumHeight[c] / count[c];
      if (height < midHeight) {
        continue; // this CELL faces down: not a scoring target
      }
      if (height > bestHeight) {
        bestHeight = height;
        best = c;
      }
    }
    if (best < 0) {
      return;
    }
    // CELLS 1 (red, 34-37) and 2 (blue, 38-41) face the audience wall; 0 (30-33) and 3 (42-45) the far wall
    AimGeometry.Aim aim = AimGeometry.solve(clusters[best], best == 1 || best == 2, cameraYawRad);
    this.hiveSeen = true;
    this.hiveTagCount = count[best];
    this.tagDistance = aim.tagDistanceM;
    this.aimCorrection = aim.correctionDeg;
    this.distance = aim.formulaDistanceM;
    this.windowDeg = aim.windowDeg;
    this.distanceCheckRatio = sumRangeFromHeight[best] / sumRange3d[best];
  }

  /** The 3D distance disagrees with the tag-height distance (check the pipeline's marker size: 82.55 mm). */
  public boolean distanceSuspect() {
    return !Double.isNaN(distanceCheckRatio) && Math.abs(distanceCheckRatio - 1) > DISTANCE_CHECK_TOLERANCE;
  }

  /** Seeing the target from closer than TOO_CLOSE_M: the formula's speed climbs steeply there. */
  public boolean isTooClose() {
    return this.hasValidTarget() && distance < TOO_CLOSE_M;
  }

  /** The turret turn (deg, + = right) the last frame asked for, before the trim and the lead. */
  public double aimCorrectionDeg() {
    return aimCorrection;
  }

  /**
   * Every loop while aiming, before {@link #updateTarget}: reads the Limelight and, on a new frame, sets the aim from
   * where the turret and the robot were when that frame was taken (TurretTracker).
   *
   * @param robotPose        Pedro pose (in, rad): the heading for the camera's field heading, x/y for the pre-aim
   * @param robotTwist       Pedro robot-frame velocity (in/s forward, in/s left, rad/s CCW)
   * @param turretEncoderDeg the turret encoder angle (Outtake.getTurretPosDegrees)
   */
  public void updateAim(Pose robotPose, Twist robotTwist, double turretEncoderDeg) {
    long now = System.nanoTime();
    if (now - lastAimCallNs > 200_000_000L) {   // a new aiming session: drop the old history and fix
      tracker.reset();
      lastFrameTimestamp = Double.NaN;
    }
    lastAimCallNs = now;
    this.turretPosition = turretEncoderDeg + currentColor.getSteadyState();
    this.headingDeg = Math.toDegrees(robotPose.heading());
    this.robotXIn = robotPose.x();
    this.robotYIn = robotPose.y();
    this.yawRateDegS = Math.toDegrees(robotTwist.omega);
    tracker.record(now, turretPosition, headingDeg);

    // Update goal; a new frame is looked at from where the turret and the robot were when it was taken
    this.currentGoal = this.limelight.getLatestResult();
    boolean newFrame = currentGoal != null && currentGoal.getTimestamp() != lastFrameTimestamp;
    double turretAtCapture = turretPosition;
    double headingAtCapture = headingDeg;
    if (newFrame) {
      lastFrameTimestamp = currentGoal.getTimestamp();
      long captureNs = currentGoal.getControlHubTimeStampNanos()
          - (long) ((currentGoal.getCaptureLatency() + currentGoal.getTargetingLatency()) * 1e6);
      double[] then = tracker.at(captureNs);
      turretAtCapture = then[0];
      headingAtCapture = then[1];
    }
    // Turret position: 180 = shooting forward, 90 = left (CW +), so the shot line is (180 - position) CCW of forward
    this.cameraYawRad = Math.toRadians(headingAtCapture + 180 - turretAtCapture);
    this.updateHiveTarget();

    // Update Velocities and Angles (robot frame: x to the right, y forward, m/s)
    double xVelocity = -robotTwist.vy * 0.0254;
    double yVelocity = robotTwist.vx * 0.0254;
    // Line of sight to the opening, as a turret position: the held aim when there is one (not the turret now + a
    // turn measured from where the turret was when the frame was taken)
    double lineOfSight = tracker.hasRecentFix(now) ? tracker.targetFromField(headingDeg)
        : turretPosition + this.aimCorrection;
    double totalAngleRad = Math.toRadians(lineOfSight + 180);

    this.vPerpendicular = xVelocity * Math.cos(totalAngleRad)
        - yVelocity * Math.sin(totalAngleRad);

    this.vParallel = xVelocity * Math.sin(totalAngleRad)
        + yVelocity * Math.cos(totalAngleRad); // figure this out

    if (newFrame && this.hasValidTarget()) {
      tracker.visionFix(now, turretAtCapture, headingAtCapture, this.calculateError());
    }
  }

  /**
   * Calculates the target velocity for the shooter based on the current teleop goal tag reading
   *
   * @return the target power for the shooter, or the last calculated power if no valid reading
   */
  public double calculateTargetVelocity() {
    if (this.hasValidTarget()) {
      boolean m58 = SHOT_MODEL_58_DEG;
      double q = (m58 ? SHOT58_A : SHOT_A) * distance - (m58 ? SHOT58_B : SHOT_B);
      if (q > 0) {
        // Shot formula from the CAD + physics model (see SHOT_* constants), plus the field trim
        double ticks = (distance * Math.pow(q, -0.5)) / (m58 ? SHOT58_K : SHOT_K) + (m58 ? SHOT58_OFFSET : SHOT_OFFSET)
            - vParallel * (m58 ? SHOT58_MOVE_GAIN : SHOT_MOVE_GAIN) + SHOT_TRIM_TICKS;
        lastCalculatedVel = 20.0 * Math.round(ticks / 20.0);
      }
    }
    return lastCalculatedVel;
  }

  private double calculateLeadAngleDegrees() {
    double tofA = SHOT_MODEL_58_DEG ? TOF58_A : TOF_A;
    double tofB = SHOT_MODEL_58_DEG ? TOF58_B : TOF_B;
    double timeOfFlight = Math.sqrt(Math.max(2 * (tofA * distance - tofB) / 9.8, 0.0025));
    return Math.toDegrees(Math.atan((vPerpendicular * timeOfFlight) / distance));
  }

  public double calculateError() {
    return this.aimCorrection + AIM_TRIM_DEG - this.calculateLeadAngleDegrees();
  }

  public boolean hasValidTarget() {
    return currentGoal != null && currentGoal.isValid() && this.hiveSeen && currentGoal.getStaleness() < STALE_MS;
  }

  /**
   * The turret target: at rest when not aiming; else the field direction of the last frames (VISION, or HOLD for
   * TurretTracker.LOST_HOLD_MS after the tags are lost) for the robot's heading now; then, in the teleop (a target
   * CELL), straight at that CELL's opening from odometry (ODOMETRY); in the autos, the pre-aim they set (LAST).
   * Limited to the turret's range (TurretTracker.limit).
   */
  public void updateTarget(double turretPos, boolean shouldAim, AllianceColor color) {
    long now = System.nanoTime();
    this.turretPosition = turretPos + color.getSteadyState();
    if (shouldAim && !wasAiming) {
      odometryPreaimSinceNs = -1;         // a new press: the odometry pre-aim may try again
    }
    wasAiming = shouldAim;

    if (!shouldAim) {
      target = color.getSteadyState();
      aimSource = "REST";
    } else if (tracker.hasRecentFix(now)) {
      odometryPreaimSinceNs = -1;         // tags seen: a later pre-aim gets its full time again
      target = tracker.targetFromField(headingDeg);
      aimSource = this.hasValidTarget() ? "VISION" : "HOLD";
      if (this.hasValidTarget()) {
        lastTarget = target;
      }
    } else if (targetCell != null && preaimAllowed(now)) {
      double[] opening = openingIn(color, targetCell);
      target = TurretTracker.targetFromBearing(headingDeg,
          TurretTracker.openingBearingDeg(robotXIn, robotYIn, Math.toRadians(headingDeg), opening[0], opening[1]));
      aimSource = "ODOMETRY";
    } else if (targetCell != null) {
      target = color.getSteadyState();    // odometry pre-aim found nothing: rest, as before (re-press RB to retry)
      aimSource = "REST-SEARCH";
    } else {
      target = lastTarget;
      aimSource = "LAST";
    }
    target = TurretTracker.limit(target, this.turretPosition);
    error = target - this.turretPosition;
  }

  /** Odometry pre-aim allowed: within ODOMETRY_PREAIM_TIMEOUT_MS of its start in this press (a fix restarts it). */
  private boolean preaimAllowed(long now) {
    if (odometryPreaimSinceNs < 0) {
      odometryPreaimSinceNs = now;
    }
    return now - odometryPreaimSinceNs <= ODOMETRY_PREAIM_TIMEOUT_MS * 1_000_000L;
  }

  /** The opening of {@code color}'s {@code cell} when it is up (in, Pedro); BLUE's is RED's rotated about the centre. */
  static double[] openingIn(AllianceColor color, HiveTargeting.Cell cell) {
    double[] red = cell == HiveTargeting.Cell.AUDIENCE ? OPENING_RED_AUDIENCE_IN : OPENING_RED_SCORING_IN;
    if (color == AllianceColor.RED) {
      return red;
    }
    // BLUE's CELL facing the same wall is RED's other CELL rotated 180 degrees
    double[] other = cell == HiveTargeting.Cell.AUDIENCE ? OPENING_RED_SCORING_IN : OPENING_RED_AUDIENCE_IN;
    return new double[]{HiveTargeting.FIELD_IN - other[0], HiveTargeting.FIELD_IN - other[1]};
  }

  /** Where the turret target comes from now: REST, VISION, HOLD, ODOMETRY, REST-SEARCH (pre-aim gave up) or LAST. */
  public String aimSource() {
    return aimSource;
  }

  /** The "aimed" threshold (deg): a fraction of the opening's window from here, capped at AIM_RGB_THRESHOLD. */
  public double aimedThresholdDeg() {
    if (Double.isNaN(windowDeg)) {
      return AIM_RGB_THRESHOLD;
    }
    return Math.min(AIM_RGB_THRESHOLD, AIM_WINDOW_FRACTION * windowDeg);
  }

  /** The turret is on the target CELL's opening closely enough to shoot (what the light shows as aimed). */
  public boolean isAimed() {
    return !"REST".equals(aimSource) && this.hasValidTarget() && Math.abs(error) < aimedThresholdDeg();
  }


  public double updateAimPID() { // returns the turn power from pid for autoaiming
    double dt = Math.min(aimTimer.seconds(), 0.1);   // the first call comes after the whole INIT wait
    aimTimer.reset();
    if ("REST".equals(aimSource)) {
      aimIntegral = 0;                                // no windup while resting
    }

    // Integral
    aimIntegral += error * dt;

    // Derivative
    double derivative = (error - aimLastError) / dt;
    derivative = Range.clip(derivative, -50, 50); // prevent explosion on large error jumps
    aimLastError = error;

    // Light first (also inside the deadband): aimed = within a fraction of the opening's window from here
    double threshold = aimedThresholdDeg();
    if ("REST".equals(aimSource)) {
      rgb.setPosition(.277);
    } else if (Math.abs(error) < threshold && this.hasValidTarget()) {
      rgb.setPosition(.5);
    } else if (this.hasValidTarget()) {
      rgb.setPosition(0.277);
    } else if (Math.abs(error) < threshold) {
      rgb.setPosition(.611);
    } else {
      rgb.setPosition(0);
    }

    // The target moves with the robot's heading when it is held on the field: feed that rate forward (not for the
    // rest direction or the autos' robot-relative pre-aim, which turn with the robot)
    boolean fieldHeld = "VISION".equals(aimSource) || "HOLD".equals(aimSource) || "ODOMETRY".equals(aimSource);
    double feedForward = fieldHeld ? AIM_Kv * yawRateDegS : 0;

    // PID Output
    if (Math.abs(error) < AIM_DEADBAND) {
      aimIntegral = 0;
      return Range.clip(feedForward, -1, 1);
    }

    double output = AIM_Kp * error
        + AIM_Ki * aimIntegral
        + AIM_Kd * derivative
        + AIM_Ks * Math.signum(error)
        + feedForward;

    // Clamp for safety
    output = Range.clip(output, -1, 1);
    return output;   // return turn power
  }


  public void setPipeline(AllianceColor color) {
    this.limelight.pipelineSwitch(color.getLLPipelineTeleOP());
  }

}
