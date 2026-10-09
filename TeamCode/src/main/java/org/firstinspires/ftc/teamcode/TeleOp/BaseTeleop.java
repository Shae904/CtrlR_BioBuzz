package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseTeleop {

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;
  double headingOffset;

  private boolean autoCalculateShootPower = true;

  public BaseTeleop(LinearOpMode opMode, Robot robot, double headingOffset) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
    this.robot.setBulkCachingMode(BulkCachingMode.MANUAL);
    this.headingOffset = Math.toRadians(headingOffset);
  }

  public void run() {
    // --- INIT ---
    // --- INIT LOOP ---
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE COLOR", robot.getAllianceColor());
      telemetry.update();
    }

    // --- START ---
    this.robot.setTiltFolded();
    while (opMode.opModeIsActive()) {
      this.robot.clearBulkCache();
      this.robot.intake.updateAutoCycle();
      this.robot.follower.update();

      // DRIVETRAIN
      double x = this.opMode.gamepad1.left_stick_x;
      double y = -this.opMode.gamepad1.left_stick_y;
      float rotStickAvg = this.opMode.gamepad1.right_stick_x + this.opMode.gamepad2.right_stick_x;

      this.robot.updateAimingSystem(
          this.opMode.gamepad1.right_bumper || this.opMode.gamepad2.right_bumper,
          this.opMode.gamepad1.square
      );

      this.fieldCentricDrive(x, y, rotStickAvg);

      // OUTTAKE
      if (this.opMode.gamepad2.triangle || this.opMode.gamepad1.triangle) {
        robot.intake.setCyclePosition(FlapperState.SHOOT);
      } else if (robot.intake.isCycleFinished()) {
        robot.intake.setCyclePosition(FlapperState.LOCKED);
      }

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

      if (this.opMode.gamepad1.dpad_up) {
        // TODO: implement ?
      }

      // TELEMETRY
      updateTelemetry();
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

    telemetry.update();
  }
}
