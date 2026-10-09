package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Iterator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.FifteenBall.BaseClose15;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;

@Configurable
public class BaseAuton {

  public static int SHOOT_TIME_QUICK = 700;
  public static int INTAKE_SPIKE_TIME = 150;
  public static int INTAKE_TIME = 150;

  public static double INTAKE_DRIVE_MAX_POWER = 1.0;
  public static double GATE_DRIVE_MAX_POWER = 1.0;


  public enum PathState {
    PRELOAD, PPG, PGP, GPP, GATE, SPIKE, CYCLE, PARK, STOP,
  }

  protected final LinearOpMode opMode;
  protected final Robot robot;
  protected final Telemetry telemetry;
  protected final Timer pathTimer;
  protected final double[] shootPos;

  protected PathState pathState = PathState.PRELOAD;
  protected Iterator<BaseClose15.PathState> pathOrder;


  public BaseAuton(LinearOpMode opMode, Robot robot, double[] shootPos) {
    this.opMode = opMode;
    this.robot = robot;
    this.telemetry = opMode.telemetry;
    this.shootPos = shootPos;
    this.pathTimer = new Timer();
  }

  public Pose poseFromArr(double[] arr) {
    return this.robot.getAllianceColor().poseFromArray(arr);
  }

  public Pose poseFromArrNonMirror(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  public void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  /**
   * Shoots three after running a PathChain. Assumes flywheel is already spun-up to speed
   *
   * @param toShootPose      PathChain that ends in the shooting Pose
   * @param intakeShootPower power to run the intake at during shooting
   * @param shootTime        how long to shoot for in milliseconds
   */
  public void shootThree(PathChain toShootPose, double intakeShootPower, int shootTime) {
    // Finish any ongoing path if one is active
    while (opMode.opModeIsActive() && (robot.follower.isBusy())) {
      robot.updateAutoControls();
    }

    // Move to shoot position
    robot.follower.followPath(toShootPose, true);
    while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.intake.isCycleFinished())) {
      robot.updateAutoControls();
    }

    // Set intake speed + open shooting game
    ElapsedTime shootTimer = new ElapsedTime();
    robot.intake.setIntakePower(intakeShootPower);
    robot.intake.setCyclePosition(FlapperState.SHOOT);

    // Shoot for shootTime ms
    while (opMode.opModeIsActive() && shootTimer.milliseconds() < shootTime) {
      robot.updateAutoControls();
    }

    // Stop the intake and close the shooting gate
    robot.intake.setIntakePower(0);
    robot.intake.setCyclePosition(FlapperState.LOCKED);
  }

  /**
   * @param preIntake        NULLABLE - path to follow before intake path. If `null` skipped
   * @param intake           path along which to intake
   * @param intakeDrivePower max power to drive the intake path with
   * @param intakeTime       time to idle intaking after completing the intake path
   */
  public void intakeThree(PathChain preIntake, PathChain intake, double intakeDrivePower, int intakeTime) {
    robot.intake.setIntakePower(Intake.POWER_INTAKE);

    if (preIntake != null) {
      robot.follower.followPath(preIntake);
      while (opMode.opModeIsActive() && robot.follower.isBusy()) {
        robot.updateAutoControls();
      }
    }

    robot.follower.followPath(intake, intakeDrivePower, false);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    ElapsedTime intakeTimer = new ElapsedTime();
    while (opMode.opModeIsActive() && intakeTimer.milliseconds() <= intakeTime) {
      robot.updateAutoControls();
    }

  }

}
