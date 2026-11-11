package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.HiveTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseTeleop {

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;
  double headingOffset;

  private boolean autoCalculateShootPower = true;

  /** Field trims per gamepad2 dpad press (auto-speed mode): flywheel ticks/s (up/down) and turret aim deg (left/right). */
  public static double SPEED_TRIM_STEP = 10;
  public static double AIM_TRIM_STEP = 0.5;

  /** Which CELL the Limelight targets: gamepad2 circle (Xbox B) toggles AUDIENCE / SCORING; no auto switching. */
  private HiveTargeting.Cell cell;
  /** Flower toggled down by gamepad1 dpad down (gamepad2 cross also holds it down). */
  private boolean flowerDown;
  private String poseSource = "";
  private final ElapsedTime pipelineMismatch = new ElapsedTime();

  public BaseTeleop(LinearOpMode opMode, Robot robot, double headingOffset) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
    this.robot.setBulkCachingMode(BulkCachingMode.MANUAL);
    this.headingOffset = Math.toRadians(headingOffset);
  }

  public void run() {
    // --- INIT ---
    robot.intake.setFlapper(false);   // DOWN before the match starts, wherever the last opmode left it
    // Field pose for the CELL targeting: where the AUTO ended, or the AUTO start spot when no AUTO ran just before
    double autoAgeS = PoseStorage.autoPoseAgeS();
    Pose start = PoseStorage.takeAutoPose();
    if (start != null) {
      poseSource = String.format("AUTO's last pose (%.0f s ago)", autoAgeS);
    } else {
      start = PoseStorage.defaultStart(robot.getAllianceColor());
      poseSource = "no AUTO just before: assumes the AUTO start spot";
    }
    robot.follower.setPose(start);
    cell = HiveTargeting.initialCell(start.y());   // the side the robot starts on; the drivers switch with square / circle
    robot.limelight.targetCell(cell);

    // --- INIT LOOP --- (the CELL buttons, the wall reset and the trims work here too)
    while (this.opMode.opModeInInit()) {
      this.robot.clearBulkCache();
      this.robot.follower.update();
      this.resetOdometryAtWall();
      this.updateCellTarget();
      this.updateTrims();
      // drop presses of the match-only buttons made during INIT (they would fire at START)
      this.opMode.gamepad1.dpadUpWasPressed();
      this.opMode.gamepad1.dpadDownWasPressed();
      this.opMode.gamepad1.dpadLeftWasPressed();
      this.opMode.gamepad1.leftBumperWasPressed();
      this.opMode.gamepad2.leftBumperWasPressed();
      telemetry.addData("ALLIANCE COLOR", robot.getAllianceColor());
      telemetry.addLine("Turret must point " + (robot.getAllianceColor() == AllianceColor.RED
          ? "LEFT" : "RIGHT") + " (its rest) when INIT is pressed: the encoder zeroes there");
      addTargetTelemetry();
      telemetry.update();
    }

    // --- START ---
    this.robot.setTiltFolded();
    while (opMode.opModeIsActive()) {
      this.robot.clearBulkCache();
      this.robot.follower.update();
      this.resetOdometryAtWall();
      this.updateCellTarget();
      this.updateTrims();

      // DRIVETRAIN
      double x = this.opMode.gamepad1.left_stick_x;
      double y = -this.opMode.gamepad1.left_stick_y;
      float rotStickAvg = this.opMode.gamepad1.right_stick_x + this.opMode.gamepad2.right_stick_x;

      this.robot.updateAimingSystem(this.opMode.gamepad1.right_bumper || this.opMode.gamepad2.right_bumper);

      this.fieldCentricDrive(x, y, rotStickAvg);

      // FLAPPER: up (feeding) only while triangle is held on either pad, down otherwise
      robot.intake.setFlapper(this.opMode.gamepad2.triangle || this.opMode.gamepad1.triangle);

      // FLOWER: gamepad1 dpad down toggles it down / up; gamepad2 cross holds it down
      if (this.opMode.gamepad1.dpadDownWasPressed()) {
        flowerDown = !flowerDown;
        this.opMode.gamepad1.rumble(100);
      }
      robot.intake.setFlower(flowerDown || this.opMode.gamepad2.cross);

      if (this.opMode.gamepad2.right_stick_button) {
        this.opMode.gamepad2.rumble(500);
        autoCalculateShootPower = true;
      } else if (this.opMode.gamepad2.left_stick_button) {
        this.opMode.gamepad2.rumble(500);
        autoCalculateShootPower = false;
      }

      boolean formulaRead = this.opMode.gamepad2.leftBumperWasPressed();   // read every loop: no stale press later
      if (this.opMode.gamepad1.dpad_up) {   // flywheel stop (was dpad down, now the flower toggle)
        robot.outtake.stop();
      } else if (autoCalculateShootPower) {
        if (this.opMode.gamepad1.right_bumper || this.opMode.gamepad2.right_bumper) {
          robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
        } else {
          robot.outtake.setTargetVelocity(1000); // 400
        }
      } else {
        if (this.opMode.gamepad2.dpad_up) {
          robot.outtake.setTargetVelocity(Outtake.farSpeed);
        } else if (this.opMode.gamepad2.dpad_down) {
          robot.outtake.setTargetVelocity(Outtake.medSpeed);
        } else if (this.opMode.gamepad2.dpad_right) {
          robot.outtake.setTargetVelocity(Outtake.cycleSpeed);
        } else if (formulaRead) {
          robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
        }
      }
      robot.outtake.updatePIDControl();

      // INTAKE
      if (this.opMode.gamepad1.right_trigger > 0.05 || this.opMode.gamepad1.left_trigger > 0.05) {
        robot.intake.setIntakePower(this.opMode.gamepad1.right_trigger - this.opMode.gamepad1.left_trigger);
      } else if (this.opMode.gamepad2.right_trigger > 0.05 || this.opMode.gamepad2.left_trigger > 0.05) {
        robot.intake.setIntakePower(this.opMode.gamepad2.right_trigger - this.opMode.gamepad2.left_trigger);
      } else {
        robot.intake.setIntakePower(0);
      }


      // TELEMETRY
      updateTelemetry();
    }
  }

  /**
   * LIMELIGHT CELL, manual, gamepad2 only: circle (Xbox B) toggles between our AUDIENCE CELL (RED 34-37 / BLUE 38-41,
   * pipeline 5 / 6) and our SCORING CELL (RED 30-33 / BLUE 42-45, pipeline 3 / 4). Same button for both alliances; the
   * SDK sets the PS and Xbox names from the same physical button, so this works on either controller.
   */
  private void updateCellTarget() {
    if (this.opMode.gamepad2.circleWasPressed()) {
      cell = cell == HiveTargeting.Cell.AUDIENCE ? HiveTargeting.Cell.SCORING : HiveTargeting.Cell.AUDIENCE;
      this.opMode.gamepad2.rumble(150);
    }
    robot.limelight.targetCell(cell);
  }

  /**
   * FIELD TRIMS (gamepad2 dpad, auto-speed mode only; in manual-speed mode the dpad keeps its speed presets):
   * up / down = flywheel +/- SPEED_TRIM_STEP ticks/s on top of the formula (Limelight.SHOT_TRIM_TICKS), right / left =
   * turret aim +/- AIM_TRIM_STEP deg (Limelight.AIM_TRIM_DEG, + = right). They last until the app restarts or new code
   * is deployed: copy the numbers from telemetry into Limelight.java once they're right.
   */
  private void updateTrims() {
    boolean up = this.opMode.gamepad2.dpadUpWasPressed();
    boolean down = this.opMode.gamepad2.dpadDownWasPressed();
    boolean right = this.opMode.gamepad2.dpadRightWasPressed();
    boolean left = this.opMode.gamepad2.dpadLeftWasPressed();
    if (!autoCalculateShootPower) {
      return;
    }
    if (up) {
      Limelight.SHOT_TRIM_TICKS += SPEED_TRIM_STEP;
    } else if (down) {
      Limelight.SHOT_TRIM_TICKS -= SPEED_TRIM_STEP;
    }
    if (right) {
      Limelight.AIM_TRIM_DEG += AIM_TRIM_STEP;
    } else if (left) {
      Limelight.AIM_TRIM_DEG -= AIM_TRIM_STEP;
    }
    if (up || down || right || left) {
      this.opMode.gamepad2.rumble(80);
    }
  }

  /**
   * ODOMETRY RESET (gamepad2 back / view): with the robot pushed against the AUDIENCE wall (anywhere along it, any face),
   * sets the odometry y to the robot centre's distance from that wall (from the footprint and the heading); x and
   * heading are kept. Only the turret's pre-aim (no tags in view) uses the odometry now.
   */
  private void resetOdometryAtWall() {
    if (!this.opMode.gamepad2.backWasPressed()) {
      return;
    }
    Pose p = robot.follower.pose();
    double y = HiveTargeting.audienceWallY(p.heading());
    robot.follower.setPose(new Pose(p.x(), y, p.heading()));
    poseSource = String.format("y reset at the audience wall (%.1f in, %s)", y,
        this.opMode.opModeInInit() ? "in INIT" : "in the match");
    this.opMode.gamepad2.rumble(400);
  }

  private void addTargetTelemetry() {
    Pose p = robot.follower.pose();
    int requested = robot.limelight.requestedPipeline();
    int running = robot.limelight.runningPipeline();
    telemetry.addData("Target CELL", "%s %s, pipeline %d  (gp2 circle/B toggles)", robot.getAllianceColor(),
        cell, requested);
    telemetry.addData("Trims", "speed %+.0f ticks/s (gp2 dpad up/down), aim %+.1f deg (dpad right/left), eta %.3f",
        Limelight.SHOT_TRIM_TICKS, Limelight.AIM_TRIM_DEG, Limelight.SHOT_ETA);
    if (robot.getAllianceColor() == AllianceColor.BLUE) {
      telemetry.addData("Blue scoring aim trim", "%+.1f deg (Panels: Limelight.AIM_TRIM_BLUE_SCORING_DEG), in use now %+.1f deg",
          Limelight.AIM_TRIM_BLUE_SCORING_DEG, robot.limelight.aimTrimDeg());
    }
    if (robot.limelight.hasValidTarget()) {
      telemetry.addData("Aim", "%s: %d tags, turn %+.1f deg, opening %.2f m, aimed within %.1f deg%s",
          robot.limelight.aimSource(), robot.limelight.hiveTagCount, robot.limelight.aimCorrectionDeg(),
          robot.limelight.distance, robot.limelight.aimedThresholdDeg(), robot.limelight.isAimed() ? " - AIMED" : "");
    } else {
      telemetry.addData("Aim", "%s: no tags of the target CELL", robot.limelight.aimSource());
    }
    if (robot.limelight.isTooClose()) {
      telemetry.addLine(String.format("TOO CLOSE: %.2f m to the opening (under %.2f): the formula's speed climbs, back up",
          robot.limelight.distance, Limelight.TOO_CLOSE_M));
    }
    if (robot.limelight.distanceSuspect()) {
      telemetry.addLine(String.format("WARNING: 3D distance is %.2fx the tag-height distance: check this pipeline's "
          + "marker size (82.55 mm) and Full 3D", 1 / robot.limelight.distanceCheckRatio));
    }
    if (robot.limelight.tagsWithout3d > 0) {
      telemetry.addLine("WARNING: " + robot.limelight.tagsWithout3d
          + " tag(s) without a 3D pose (ignored): turn on Full 3D in this Limelight pipeline");
    }
    telemetry.addData("Turret error", robot.limelight.error);   // a number: plot it in Panels' Graph to tune AIM_*
    telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    telemetry.addData("Pose from", "%s  (gp2 back = reset y at the audience wall)", poseSource);
    if (running == requested || running < 0) {
      pipelineMismatch.reset();
    } else if (pipelineMismatch.seconds() > 1.0) {
      telemetry.addLine("WARNING: Limelight is on pipeline " + running + ", not " + requested
          + ": in the Limelight UI click 'Stop Ignoring NetworkTables Index'");
    }
  }

  private void fieldCentricDrive(double x, double y, double rx) {
    if (this.opMode.gamepad1.leftBumperWasPressed()) {
      robot.imu.resetYaw();
      this.headingOffset = 0;
    }

    // Field Centric Drive
    double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + this.headingOffset;

    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
    rotX = rotX * 1.1;  // Counteract imperfect strafing

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;

    robot.fr.setPower(frontRightPower);
    robot.fl.setPower(frontLeftPower);
    robot.br.setPower(backRightPower);
    robot.bl.setPower(backLeftPower);
  }

  private void updateTelemetry() {

    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity());
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());
    telemetry.addData("Vel Setpoint", robot.outtake.getSetpoint());   // = Vel Target except during the soft start
    telemetry.addData("At Target", robot.outtake.atTarget());
    addTargetTelemetry();

    telemetry.update();
  }
}
