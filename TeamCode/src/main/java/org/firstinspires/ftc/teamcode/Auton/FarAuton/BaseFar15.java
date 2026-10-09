package org.firstinspires.ftc.teamcode.Auton.FarAuton;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Auton.BaseAuton;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseFar15 extends BaseAuton {


  public static int CYCLE_LIMIT = 5;
  public static double PRELOAD_SHOOT_TIME = 800;

  public static double[] START_RED = {88, 8, 90};

  public static double[] SHOOT_RED_AFTER = {90, 15, 0};

  public static double[] INTAKE_HP_START_RED = {118, 9, 0};
  public static double[] INTAKE_HP_MIDDLE_RED = {131, 9, 0};
  public static double[] INTAKE_HP_CONTROL_RED = {100, 16, 0};
  public static double[] INTAKE_HP_END_RED = {137, 23, 0};
  public static double[] PARK_POS_RED = {105, 37, 0};

  public static double[] INTAKE_SPIKE_START_RED = {105, 35, 0};
  public static double[] INTAKE_SPIKE_END_RED = {135, 35, 0};

  PathChain shootPreLoad,
      preIntakeHP, intakeHP, shootHP,
      intakeSpike, shootSpike,
      parkPath;

  ElapsedTime globalTimer = new ElapsedTime();
  private boolean spikePath = false;
  int cycleCounter = 0;

  public BaseFar15(LinearOpMode opMode, Robot robot, double[] shootPos) {
    super(opMode, robot, shootPos);
  }

  void buildPaths() {
    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(START_RED), poseFromArrNonMirror(shootPos)))
        .setLinearHeadingInterpolation(
            poseFromArr(START_RED).getHeading(),
            poseFromArrNonMirror(shootPos).getHeading()
        )
        .setTimeoutConstraint(300)
        .build();

    preIntakeHP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_HP_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_HP_START_RED).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    intakeSpike = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_SPIKE_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_SPIKE_START_RED).getHeading()
        )
        .addPath(new BezierLine(poseFromArr(INTAKE_SPIKE_START_RED), poseFromArr(INTAKE_SPIKE_END_RED)))
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_SPIKE_START_RED).getHeading(),
            poseFromArr(INTAKE_SPIKE_END_RED).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    shootSpike = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_SPIKE_END_RED), poseFromArr(SHOOT_RED_AFTER)))
        /*.setLinearHeadingInterpolation(
            poseFromArr(INTAKE_SPIKE_END_RED).getHeading(),
            poseFromArr(SHOOT_RED_AFTER).getHeading()
        )*/
        .setTangentHeadingInterpolation()
        .setReversed()
        .setTimeoutConstraint(50)
        .build();

    intakeHP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_HP_START_RED), poseFromArr(INTAKE_HP_MIDDLE_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_HP_START_RED).getHeading())
        .addPath(new BezierCurve(
            poseFromArr(INTAKE_HP_MIDDLE_RED),
            poseFromArr(INTAKE_HP_CONTROL_RED),
            poseFromArr(INTAKE_HP_END_RED))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_HP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    shootHP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_HP_END_RED), poseFromArr(SHOOT_RED_AFTER)))
        /*.setLinearHeadingInterpolation(
            poseFromArr(INTAKE_HP_END_RED).getHeading(),
            poseFromArr(SHOOT_RED_AFTER).getHeading()
        )*/
        .setTangentHeadingInterpolation()
        .setReversed()
        .setTimeoutConstraint(300)
        .build();

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT_RED_AFTER), poseFromArr(PARK_POS_RED)))
        .setLinearHeadingInterpolation(
            poseFromArr(shootPos).getHeading(),
            poseFromArr(PARK_POS_RED).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();
  }

  public void autonomousPathUpdate() {
    if (globalTimer.seconds() >= 27 && pathState != PathState.PARK && pathState != PathState.STOP) {
      pathState = PathState.PARK;
    }

    switch (pathState) {
      case PRELOAD:
        robot.follower.followPath(shootPreLoad);
        while (opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }

        robot.limelight.updateAim(0, 0);
        robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
        do {
          robot.updateAutoControls();
        } while (opMode.opModeIsActive() && !robot.outtake.atTarget());
        robot.intake.setCyclePosition(FlapperState.SHOOT);

        ElapsedTime preloadTimer = new ElapsedTime();
        while (opMode.opModeIsActive() && preloadTimer.milliseconds() < PRELOAD_SHOOT_TIME) {
          robot.updateAutoControls();
        }
        robot.intake.setCyclePosition(FlapperState.LOCKED);

        setPathState(pathOrder.next());
        break;

      case SPIKE:
        intakeThree(null, intakeSpike, INTAKE_DRIVE_MAX_POWER, INTAKE_TIME);
        shootThree(shootSpike, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case CYCLE:
        cycleCounter++;
        intakeThree(preIntakeHP, intakeHP, INTAKE_DRIVE_MAX_POWER, INTAKE_TIME);
        shootThree(shootHP, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);

        if (cycleCounter >= CYCLE_LIMIT) {
          setPathState(pathOrder.next());
        }
        break;

      case PARK:
        robot.follower.followPath(parkPath);
        while (opMode.opModeIsActive() && robot.follower.isBusy()) {
          robot.updateAutoControls();
        }
        setPathState(pathOrder.next());
        break;

      case STOP:
        robot.intake.setIntakePower(0);
        robot.outtake.setTargetVelocity(0);
        robot.intake.setCyclePosition(FlapperState.LOCKED);
        break;
    }
  }

  public void run() {
    // INIT
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    while (this.opMode.opModeInInit() && !(opMode.gamepad1.start && opMode.gamepad2.start)) {
      if (this.opMode.gamepad1.squareWasPressed()) {
        this.spikePath = !this.spikePath;
      }

      telemetry.addData("(SQUARE) | INTAKE FROM SPIKE?", this.spikePath);
      telemetry.addLine("Press START on BOTH controllers to lock-in configuration.");
      telemetry.update();
    }

    while (this.opMode.opModeInInit()) {
      telemetry.addLine("== CONFIGURATION LOCKED ==");
      telemetry.addData("INTAKE FROM SPIKE?", this.spikePath);
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    globalTimer.reset();
    robot.follower.setStartingPose(poseFromArr(START_RED));
    robot.outtake.setTargetVelocity(Outtake.farSpeed);
    robot.intake.setIntakePower(Intake.POWER_INTAKE);
    robot.limelight.setTarget(180.0);

    List<PathState> paths = new ArrayList<>(List.of(PathState.CYCLE, PathState.PARK, PathState.STOP));

    if (this.spikePath) {
      paths.add(0, PathState.SPIKE);
    }
    pathOrder = paths.iterator();

    // LOOP
    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Current Cycle", cycleCounter);
      telemetry.addData("Global Timer", globalTimer.seconds());
      telemetry.update();
    }
  }

}
