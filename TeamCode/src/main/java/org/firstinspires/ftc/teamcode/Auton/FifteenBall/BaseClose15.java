package org.firstinspires.ftc.teamcode.Auton.FifteenBall;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.Auton.BaseAuton;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Configurable
public class BaseClose15 extends BaseAuton {

  public static double INTAKE_TIMER_GATE = 800;
  public static double INTAKE_TIMER_MOVE = 150;
  public static int CYCLE_LIMIT = 4;

  public static double[] START_RED = {114, 130, 39}; // 114.25, 130, 180

  public static double[] SHOOT_AFTER = {87, 82, 0}; // 114.25, 130, 180


  public static double[] INTAKE_PPG_START_RED = {89, 86, 0};
  public static double[] INTAKE_PPG_END_RED = {122, 86, 0};

  public static double[] INTAKE_PGP_START_RED = {89, 60, 0};
  public static double[] INTAKE_PGP_END_RED = {125, 60, 0};

  public static double[] INTAKE_PGP_MIDDLE_RED = {119, 60, 0};


  public static double[] OPEN_GATE_START = {116, 76, 0};
  public static double[] OPEN_GATE_END = {121, 74, 0};


  public static double[] INTAKE_CLASSIFIER = {126, 62.5, 25};
  public static double[] INTAKE_CLASSIFIER_TWO = {128.5, 54.5, 35}; //130.5

  public static double[] PARK_POS = {94, 65, 0};

  int cycleCounter = 0;

  PathChain
      shootPreLoad,
      intakePPG, openGatePPG,
      intakePGP, openGatePGP,
      intakeClassifier, intakeClassifierTwo,
      shootGate, shootGateIntake,
      parkPath;

  public BaseClose15(LinearOpMode opMode, Robot robot, double[] shootPos) {
    super(opMode, robot, shootPos);
  }

  void buildPaths() {
    shootPreLoad = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(START_RED), poseFromArrNonMirror(shootPos)))
        .setTangentHeadingInterpolation()
        .setReversed()
        .setTimeoutConstraint(300)
        .build();

    intakePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArrNonMirror(shootPos), poseFromArr(INTAKE_PPG_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArrNonMirror(shootPos).getHeading(),
            poseFromArr(INTAKE_PPG_START_RED).getHeading()
        )
        .addPath(new BezierLine(poseFromArr(INTAKE_PPG_START_RED), poseFromArr(INTAKE_PPG_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    openGatePPG = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(INTAKE_PPG_END_RED),
            poseFromArr(OPEN_GATE_START))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PPG_END_RED).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setConstantHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading())
        .build();

    intakePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(shootPos), poseFromArr(INTAKE_PGP_START_RED)))
        .setLinearHeadingInterpolation(
            poseFromArr(SHOOT_AFTER).getHeading(),
            poseFromArr(INTAKE_PGP_START_RED).getHeading()
        )
        .addPath(new BezierLine(poseFromArr(INTAKE_PGP_START_RED), poseFromArr(INTAKE_PGP_END_RED)))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .addPath((new BezierLine(poseFromArr(INTAKE_PGP_END_RED), poseFromArr(INTAKE_PGP_MIDDLE_RED))))
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_START_RED).getHeading())
        .setTimeoutConstraint(50)
        .build();

    openGatePGP = robot.follower.pathBuilder()
        .addPath(new BezierLine(
            poseFromArr(INTAKE_PGP_MIDDLE_RED),
            poseFromArr(OPEN_GATE_START))
        )
        .setConstantHeadingInterpolation(poseFromArr(INTAKE_PGP_END_RED).getHeading())
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_START), poseFromArr(OPEN_GATE_END)))
        .setConstantHeadingInterpolation(poseFromArr(OPEN_GATE_START).getHeading())
        .build();

    intakeClassifier = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT_AFTER), poseFromArr(INTAKE_CLASSIFIER)))
        .setLinearHeadingInterpolation(
            poseFromArr(SHOOT_AFTER).getHeading(), // TODO TEST FOR BLUE
            poseFromArr(INTAKE_CLASSIFIER).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    intakeClassifierTwo = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_CLASSIFIER), poseFromArr(INTAKE_CLASSIFIER_TWO)))
        .setLinearHeadingInterpolation(
            poseFromArr(INTAKE_CLASSIFIER).getHeading(),
            poseFromArr(INTAKE_CLASSIFIER_TWO).getHeading()
        )
        .setTimeoutConstraint(50)
        .build();

    parkPath = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(SHOOT_AFTER), poseFromArr(PARK_POS)))
        .setTangentHeadingInterpolation()
        .setReversed()
        .setTimeoutConstraint(50)
        .build();

    shootGate = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(OPEN_GATE_END), poseFromArr(SHOOT_AFTER)))
        /*.setConstantHeadingInterpolation(poseFromArr(OPEN_GATE_END).getHeading())*/
        .setTangentHeadingInterpolation()
        .setReversed()
        .setTimeoutConstraint(300)
        .build();

    shootGateIntake = robot.follower.pathBuilder()
        .addPath(new BezierLine(poseFromArr(INTAKE_CLASSIFIER_TWO), poseFromArr(SHOOT_AFTER)))
        /*.setConstantHeadingInterpolation(poseFromArr(INTAKE_CLASSIFIER_TWO).getHeading())*/
        .setTangentHeadingInterpolation()
        .setReversed()
        .setTimeoutConstraint(300)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case PRELOAD:
        shootThree(shootPreLoad, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case PGP:
        intakeThree(null, intakePGP, INTAKE_DRIVE_MAX_POWER, INTAKE_SPIKE_TIME);
        robot.follower.followPath(openGatePGP, GATE_DRIVE_MAX_POWER, true);
        shootThree(shootGate, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case PPG:
        intakeThree(null, intakePPG, INTAKE_DRIVE_MAX_POWER, INTAKE_SPIKE_TIME);
        robot.follower.followPath(openGatePPG, GATE_DRIVE_MAX_POWER, true);
        shootThree(shootGate, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
        setPathState(pathOrder.next());
        break;

      case GATE:
        cycleCounter++;
        intakeGate(intakeClassifier, intakeClassifierTwo);
        shootThree(shootGateIntake, Intake.POWER_INTAKE, SHOOT_TIME_QUICK);
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

  private void intakeGate(PathChain shootToIntake, PathChain intakeToIntakeTwo) {
    ElapsedTime gateIntakeTimer = new ElapsedTime();
    robot.intake.setIntakePower(Intake.POWER_INTAKE);

    robot.follower.followPath(shootToIntake, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    gateIntakeTimer.reset();
    while (opMode.opModeIsActive() && gateIntakeTimer.milliseconds() <= INTAKE_TIMER_MOVE) {
      robot.updateAutoControls();
    }

    robot.follower.followPath(intakeToIntakeTwo, true);
    while (opMode.opModeIsActive() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

    gateIntakeTimer.reset();
    while (opMode.opModeIsActive() && gateIntakeTimer.milliseconds() <= INTAKE_TIMER_GATE) {
      robot.updateAutoControls();
    }
  }

  public void run() {
    // INIT
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    this.opMode.waitForStart();
    telemetry.addData("ALLIANCE", robot.getAllianceColor());
    telemetry.update();

    // START
    robot.follower.setStartingPose(poseFromArr(START_RED));
    robot.outtake.setTargetVelocity(Outtake.medSpeed);
    robot.intake.setIntakePower(1);
    robot.limelight.setTarget(180.0);

    pathOrder = List.of(PathState.PPG, PathState.PGP, PathState.GATE, PathState.PARK, PathState.STOP).iterator();

    // LOOP
    while (this.opMode.opModeIsActive()) {

      this.robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Current Cycle", cycleCounter);
      telemetry.update();
    }
  }

}
