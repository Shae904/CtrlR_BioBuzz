package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.HiveTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseTeleop {

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;
  double headingOffset;

  private boolean autoCalculateShootPower = true;

  /** Which CELL the Limelight targets: from odometry, or the cross (AUDIENCE) / circle (SCORING) buttons. */
  private HiveTargeting targeting;
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
    targeting = new HiveTargeting(start.y());
    robot.limelight.targetCell(targeting.cell());

    // --- INIT LOOP --- (the CELL buttons work here too)
    while (this.opMode.opModeInInit()) {
      this.robot.clearBulkCache();
      this.robot.follower.update();
      this.updateCellTarget();
      telemetry.addData("ALLIANCE COLOR", robot.getAllianceColor());
      telemetry.addData("Start pose", poseSource);
      addTargetTelemetry();
      telemetry.update();
    }

    // --- START ---
    this.robot.setTiltFolded();
    while (opMode.opModeIsActive()) {
      this.robot.clearBulkCache();
      this.robot.follower.update();
      this.updateCellTarget();

      // DRIVETRAIN
      double x = this.opMode.gamepad1.left_stick_x;
      double y = -this.opMode.gamepad1.left_stick_y;
      float rotStickAvg = this.opMode.gamepad1.right_stick_x + this.opMode.gamepad2.right_stick_x;

      this.robot.updateAimingSystem(
          this.opMode.gamepad1.right_bumper || this.opMode.gamepad2.right_bumper,
          this.opMode.gamepad1.square
      );

      this.fieldCentricDrive(x, y, rotStickAvg);

      // FLAPPER: up (feeding) only while triangle is held on either pad, down otherwise
      robot.intake.setFlapper(this.opMode.gamepad2.triangle || this.opMode.gamepad1.triangle);

      if (this.opMode.gamepad2.right_stick_button) {
        this.opMode.gamepad2.rumble(500);
        autoCalculateShootPower = true;
      } else if (this.opMode.gamepad2.left_stick_button) {
        this.opMode.gamepad2.rumble(500);
        autoCalculateShootPower = false;
      }

      if (this.opMode.gamepad1.dpad_down) {
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
        } else if (this.opMode.gamepad2.leftBumperWasPressed()) {
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

      if (this.opMode.gamepad1.dpadUpWasPressed()) {
        robot.setTiltDeployed();
      } else if (this.opMode.gamepad1.dpadLeftWasPressed()) {
        robot.setTiltFolded();
      }

      // TELEMETRY
      updateTelemetry();
    }
  }

  /**
   * LIMELIGHT CELL: automatic from the odometry y (the two tile rows along the audience wall = AUDIENCE CELL, along
   * the far wall = SCORING CELL, the middle rows keep the last one); cross = AUDIENCE, circle = SCORING on either
   * gamepad override it until odometry puts the robot in the outer tiles of the chosen side.
   */
  private void updateCellTarget() {
    // read all four so no press is left over for a later loop
    boolean audience1 = this.opMode.gamepad1.crossWasPressed();
    boolean audience2 = this.opMode.gamepad2.crossWasPressed();
    boolean scoring1 = this.opMode.gamepad1.circleWasPressed();
    boolean scoring2 = this.opMode.gamepad2.circleWasPressed();
    if (audience1 || scoring1) {
      this.opMode.gamepad1.rumble(150);
    }
    if (audience2 || scoring2) {
      this.opMode.gamepad2.rumble(150);
    }
    HiveTargeting.Cell cell = targeting.update(robot.follower.pose().y(), audience1 || audience2,
        scoring1 || scoring2);
    robot.limelight.targetCell(cell);
  }

  private void addTargetTelemetry() {
    Pose p = robot.follower.pose();
    int requested = robot.limelight.requestedPipeline();
    int running = robot.limelight.runningPipeline();
    telemetry.addData("Target CELL", "%s %s, pipeline %d (%s)", robot.getAllianceColor(), targeting.cell(), requested,
        targeting.isAuto() ? "auto" : "manual until the " + targeting.cell().name().toLowerCase() + " tiles");
    telemetry.addData("Zone", "%s (y %.1f in)", targeting.zone(), p.y());
    telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
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
    telemetry.addData("At Target", robot.outtake.atTarget());
    addTargetTelemetry();

    telemetry.update();
  }
}
