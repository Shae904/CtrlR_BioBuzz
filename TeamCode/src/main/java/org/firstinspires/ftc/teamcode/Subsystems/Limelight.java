package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
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

  public static double AIM_RGB_THRESHOLD = 4;

  // --- BIOBUZZ AprilTags (manual 9.9 / Figure 9-17): 16 tags, 4 per CELL, on each CELL's underside ---
  //   red scoring 30-33, red audience 34-37, blue audience 38-41, blue scoring 42-45
  // Heights/offsets come from the official field CAD (floor = top of tiles).
  public static double LENS_HEIGHT_M = 0.427;          // Limelight lens above floor (turret CAD on Worlds Bot)
  public static double CAM_PITCH_DEG = 35.0;           // Limelight lens axis above horizontal (new tilted mount; was level)
  public static double UP_CELL_TAG_HEIGHT_M = 1.261;   // tag cluster height when its CELL faces up (the scoring target)
  public static double DOWN_CELL_TAG_HEIGHT_M = 0.905; // tag cluster height when its CELL faces down
  public static double OPENING_BEYOND_TAGS_M = 0.152;  // CELL opening centre is this much farther (horizontally) than its tags

  // --- Shot formula from the CAD + physics model (vibecadded-ftc/shooter-model/DERIVATION.md) ---
  // ticks/s = d * (SHOT_A * d - SHOT_B)^-0.5 / SHOT_K + SHOT_OFFSET, d = horizontal camera -> CELL opening (m)
  // Valid ~1.4-3.6 m, 2.85 in POLLEN, ghettoturret hood. Calibrate SHOT_OFFSET with a couple of shots.
  public static double SHOT_A = 0.160596408922;
  public static double SHOT_B = 0.148979591837;
  public static double SHOT_K = 0.0028296709921;
  public static double SHOT_OFFSET = -315;
  public static double SHOT_MOVE_GAIN = 93 * Math.cos(Math.toRadians(50)); // ticks per m/s toward the goal (DECODE tuning)
  // POLLEN time of flight to the CELL opening: sqrt(2 * (TOF_A * d - TOF_B) / 9.8) seconds
  public static double TOF_A = 1.400;
  public static double TOF_B = 1.43;


  public final Limelight3A limelight;
  private final AllianceColor currentColor;
  private final ElapsedTime aimTimer = new ElapsedTime();

  public final Servo rgb;

  private LLResult currentGoal;

  private double lastCalculatedVel = Outtake.medSpeed;
  private double aimIntegral = 0;
  private double aimLastError = 0;

  private double vPerpendicular;
  private double vParallel;

  // Result of the multi-tag hive detection (see updateAim)
  private boolean hiveSeen = false;
  private double aimTx = 0;          // mean tx of the upward CELL's visible tags (deg)
  public double tagDistance;         // horizontal camera -> tag cluster (m)
  public int hiveTagCount;           // how many of our hive's tags were used

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
    this.limelight.pipelineSwitch(this.currentColor.getLLPipelineTeleOP());

    this.lastTarget = this.currentColor.getSteadyState();
    this.turretPosition = this.currentColor.getSteadyState();

    this.rgb = hardwareMap.servo.get("rgb");
    this.rgb.setPosition(.5);
  }

  public void setTarget(double pos) {
    this.lastTarget = pos;
  }


  /** True if the tag belongs to our alliance's HIVE (both of its CELLS). */
  private static boolean isOurHiveTag(int id, AllianceColor color) {
    return color == AllianceColor.RED ? (id >= 30 && id <= 37) : (id >= 38 && id <= 45);
  }

  /**
   * Uses every visible tag of our HIVE (not just the Limelight's primary target): groups them by CELL, picks the
   * upward-facing CELL (the higher cluster) and averages distance and tx over all of its visible tags.
   */
  private void updateHiveTarget() {
    this.hiveSeen = false;
    this.hiveTagCount = 0;
    if (currentGoal == null || !currentGoal.isValid()) {
      return;
    }
    double pitch = Math.toRadians(CAM_PITCH_DEG);
    double[] sumDist = new double[4];
    double[] sumTx = new double[4];
    double[] sumHeight = new double[4];
    double[] sumTy = new double[4];
    int[] count = new int[4];
    int[] count3d = new int[4];
    for (FiducialResult tag : currentGoal.getFiducialResults()) {
      int id = tag.getFiducialId();
      if (!isOurHiveTag(id, currentColor)) {
        continue;
      }
      int cell = (id - 30) / 4; // 0: red scoring, 1: red audience, 2: blue audience, 3: blue scoring
      // Ray to the tag, rotated from the tilted camera into a level frame
      // (camera space: +x right, +y down, +z out of the lens; tx/ty are pinhole angles)
      double tanTx = Math.tan(Math.toRadians(tag.getTargetXDegrees()));
      double tanTy = Math.tan(Math.toRadians(tag.getTargetYDegrees()));
      double rayFwd = Math.cos(pitch) - tanTy * Math.sin(pitch);
      double rayUp = Math.sin(pitch) + tanTy * Math.cos(pitch);
      double rayHoriz = Math.hypot(tanTx, rayFwd);
      // True horizontal bearing (what tx meant when the camera was level, so aim tuning still applies)
      double bearing = Math.toDegrees(Math.atan2(tanTx, rayFwd));
      double dist = Double.NaN;
      Pose3D pose = tag.getTargetPoseCameraSpace();
      if (pose != null) {
        Position p = pose.getPosition().toUnit(DistanceUnit.METER);
        if (p.z > 0.05) {
          double fwd = p.z * Math.cos(pitch) + p.y * Math.sin(pitch);
          double up = p.z * Math.sin(pitch) - p.y * Math.cos(pitch);
          dist = Math.hypot(p.x, fwd);
          sumHeight[cell] += LENS_HEIGHT_M + up;
          count3d[cell]++;
        }
      }
      if (Double.isNaN(dist)) { // no 3D pose: assume this cluster is on the upward CELL
        if (rayUp <= 0.01) {
          continue;
        }
        dist = (UP_CELL_TAG_HEIGHT_M - LENS_HEIGHT_M) * rayHoriz / rayUp;
      }
      sumDist[cell] += dist;
      sumTx[cell] += bearing;
      sumTy[cell] += tag.getTargetYDegrees();
      count[cell]++;
    }

    // Upward-facing CELL = the higher cluster (measured height when 3D is available, else larger ty)
    double midHeight = (UP_CELL_TAG_HEIGHT_M + DOWN_CELL_TAG_HEIGHT_M) / 2.0;
    int best = -1;
    double bestScore = Double.NEGATIVE_INFINITY;
    for (int c = 0; c < 4; c++) {
      if (count[c] == 0) {
        continue;
      }
      double score;
      if (count3d[c] > 0) {
        double height = sumHeight[c] / count3d[c];
        if (height < midHeight) {
          continue; // this CELL faces down: not a scoring target
        }
        score = 1000 + height;
      } else {
        score = sumTy[c] / count[c];
      }
      if (score > bestScore) {
        bestScore = score;
        best = c;
      }
    }
    if (best < 0) {
      return;
    }
    this.hiveSeen = true;
    this.hiveTagCount = count[best];
    this.tagDistance = sumDist[best] / count[best];
    this.aimTx = sumTx[best] / count[best];
    // Tags are only visible from the side the opening faces away from, so the opening is beyond them
    this.distance = this.tagDistance + OPENING_BEYOND_TAGS_M;
  }

  public void updateAim(double xVelocity, double yVelocity) {
    // Update goal
    this.currentGoal = this.limelight.getLatestResult();
    this.updateHiveTarget();

    // Update Velocities and Angles
    double txRad = Math.toRadians(this.aimTx);

    double turretRad = Math.toRadians(turretPosition + 180);
    // Total angle = turret heading + tx offset from turret center
    double totalAngleRad = turretRad + txRad;

    this.vPerpendicular = xVelocity * Math.cos(totalAngleRad)
        - yVelocity * Math.sin(totalAngleRad);

    this.vParallel = xVelocity * Math.sin(totalAngleRad)
        + yVelocity * Math.cos(totalAngleRad); // figure this out
  }

  /**
   * Calculates the target velocity for the shooter based on the current teleop goal tag reading
   *
   * @return the target power for the shooter, or the last calculated power if no valid reading
   */
  public double calculateTargetVelocity() {
    if (this.hasValidTarget()) {
      double q = SHOT_A * distance - SHOT_B;
      if (q > 0) {
        // Shot formula from the CAD + physics model (see SHOT_* constants)
        double ticks = (distance * Math.pow(q, -0.5)) / SHOT_K + SHOT_OFFSET
            - vParallel * SHOT_MOVE_GAIN;
        lastCalculatedVel = 20.0 * Math.round(ticks / 20.0);
      }
    }
    return lastCalculatedVel;
  }

  private double calculateLeadAngleDegrees() {
    double timeOfFlight = Math.sqrt(Math.max(2 * (TOF_A * distance - TOF_B) / 9.8, 0.0025));
    return Math.toDegrees(Math.atan((vPerpendicular * timeOfFlight) / distance));
  }

  public double calculateError() {
    return this.aimTx - (currentColor.getAimPose() + this.calculateLeadAngleDegrees());
  }

  public boolean hasValidTarget() {
    return currentGoal != null && currentGoal.isValid() && this.hiveSeen;
  }

  public void updateTarget(double turretPos, boolean shouldAim, AllianceColor color) {
    this.turretPosition = turretPos + color.getSteadyState();

    if (this.hasValidTarget() && shouldAim) {
      double rawTarget = this.turretPosition + this.calculateError();
      // wrap
      rawTarget = ((rawTarget % 360) + 360) % 360;

      if (Math.abs(error) < 45) {
        target = rawTarget;
        lastTarget = target;
      }

    } else if (shouldAim && !this.hasValidTarget()) {
      target = lastTarget;

    } else if (!shouldAim) {
      target = color.getSteadyState();
    }

    error = target - this.turretPosition;
  }


  public double updateAimPID() { // returns the turn power from pid for autoaiming
    double dt = aimTimer.seconds();
    aimTimer.reset();

    // Integral
    aimIntegral += error * dt;

    // Derivative
    double derivative = (error - aimLastError) / dt;
    derivative = Range.clip(derivative, -50, 50); // prevent explosion on large error jumps
    aimLastError = error;

    // PID Output

    if (Math.abs(error) < AIM_DEADBAND) {
      aimIntegral = 0;
      return 0;
    }

    if (target == this.currentColor.getSteadyState()) {
      rgb.setPosition(.277);
    } else if (Math.abs(error) < AIM_RGB_THRESHOLD && this.hasValidTarget()) {
      rgb.setPosition(.5);
    } else if (Math.abs(error) > AIM_RGB_THRESHOLD && this.hasValidTarget()) {
      rgb.setPosition(0.277);
    } else if (Math.abs(error) < AIM_RGB_THRESHOLD && !this.hasValidTarget()) {
      rgb.setPosition(.611);
    } else {
      rgb.setPosition(0);
    }

    double output = AIM_Kp * error
        + AIM_Ki * aimIntegral
        + AIM_Kd * derivative
        + AIM_Ks * Math.signum(error);

    // Clamp for safety
    output = Range.clip(output, -1, 1);
    return output;   // return turn power
  }


  public void setPipeline(AllianceColor color) {
    this.limelight.pipelineSwitch(color.getLLPipelineTeleOP());
  }

}
