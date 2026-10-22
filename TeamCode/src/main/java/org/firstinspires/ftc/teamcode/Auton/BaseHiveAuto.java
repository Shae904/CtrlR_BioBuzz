package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Auton.Hive4.BaseHive4;
import org.firstinspires.ftc.teamcode.Auton.TwoTip.BaseTwoTip;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

/**
 * What the BIOBUZZ autonomous routines ({@link BaseHive4}, {@link BaseTwoTip}) share: alliance poses, driving with a
 * timeout, the team's Limelight turret aim with that spot's aim point, and shooting through the gate.
 *
 * <p>Written and tested in the biobuzz-sim simulator (it runs this TeamCode unchanged). The numbers that came from the
 * simulator and need checking on the real robot: the aim points (BaseHive4.AIM_TX_DEG, BaseTwoTip.AIM_TX_A_DEG /
 * AIM_TX_B_DEG: tune from where the first shot lands), the feed timing (FEED_FIRST_MS, FEED_INTERVAL_MS,
 * FEED_MARGIN_MS: how long the intake must run for exactly one ball to reach the flywheel) and the GARDEN sweep /
 * flower pickup in BaseTwoTip. All of them are @Configurable (Panels).
 *
 * <p>Poses are written for RED in the Pedro field frame (inches, degrees, origin at the red/audience corner); BLUE runs
 * the same poses rotated 180 degrees about the field centre (the field is rotationally symmetric, not mirrored). A
 * shooting heading is chosen so the CELL sits in the turret's rest direction (RED: the robot's left, BLUE: its right),
 * and the turret holds that direction until the Limelight sees our upward CELL: the tags are in view as the robot
 * arrives, and the turret never chases a CELL behind the robot across its +-180 deg limit (on the 2 TIP route that
 * unwinding left it swinging +-20 deg when the shots went).
 *
 * <p>The gate is the flapper ({@link Intake#setFlapper}): down (shut) the intake collects into the hopper, up (open)
 * the intake feeds the hopper into the flywheel.
 *
 * <p>Bulk reads: MANUAL with the cache cleared at the top of every loop, like BaseTeleop. (With the old three-wheel
 * localizer, Robot's AUTO mode read one pod a loop late and put the robot 4-5 in off over the 2 TIP route; the
 * Pinpoint reports its pose over I2C, outside the bulk read, but MANUAL still keeps the motor reads consistent.)
 */
@Configurable
public abstract class BaseHiveAuto {

  /** Field interior (in): BLUE poses are RED's rotated about (FIELD_IN / 2, FIELD_IN / 2). */
  public static double FIELD_IN = 141.05;

  /** Flywheel target while driving (ticks/s), until the Limelight gives the formula's (~1140 at the shooting spots). */
  public static double SPINUP_TICKS = 1150;
  /**
   * Ready to shoot: turret within this of the tags' aim point (deg) and the flywheel within this of its target, held
   * for READY_HOLD_MS, so a turret swinging through the aim point is not ready. The CELL opening is ~20 in wide and 2
   * deg is 5 cm at 1.45 m; in the simulator the aim PID hunts about +-1.3 deg, so 1 deg made every ball wait ~1 s.
   */
  public static double AIM_TOLERANCE_DEG = 2.0;
  public static double FLYWHEEL_TOLERANCE = 30;
  public static int READY_HOLD_MS = 80;
  /** Give up waiting for "ready" after this long at the shooting spot (ms). */
  public static int AIM_TIMEOUT_MS = 3000;
  /** {@link #driveTo}: when Pedro stopped further than this from the end (in), a short straight path onto it. */
  public static double SETTLE_IN = 1.0;
  public static int SETTLE_TIMEOUT_MS = 1200;

  /** Gate travel before the intake starts feeding (ms). */
  public static int GATE_OPEN_MS = 200;
  /** With the gate open, the first ball reaches the flywheel this long after the intake starts, then one every FEED_INTERVAL_MS. */
  public static int FEED_FIRST_MS = 150;
  public static int FEED_INTERVAL_MS = 233;
  /** Feeding stops this long after the last wanted ball: under half a feed interval, so the next one stays (ms). */
  public static int FEED_MARGIN_MS = 110;
  /** {@link #shootVolley()}: intake running with the gate open while the whole hopper goes (ms). */
  public static int SHOOT_ALL_MS = 1600;
  /** {@link #shoot(int)}: longest wait for "ready" before each ball (ms). */
  public static int SHOT_READY_TIMEOUT_MS = 1000;
  public static double INTAKE_POWER = Intake.POWER_INTAKE;

  protected final LinearOpMode opMode;
  protected final Robot robot;
  protected final Telemetry telemetry;
  protected String phase = "INIT";
  private double aimTxDeg;
  private boolean aiming = true;
  private boolean readyNow;
  private final ElapsedTime readyFor = new ElapsedTime();

  protected BaseHiveAuto(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.robot = robot;
    this.telemetry = opMode.telemetry;
  }

  public abstract void run();

  /** INIT: the team's auton init plus the gate shut, telemetry until START. False when stopped before START. */
  protected boolean initAndWaitForStart(String description) {
    robot.setBulkCachingMode(BulkCachingMode.MANUAL);
    robot.initAuton();
    closeGate();
    while (opMode.opModeInInit()) {
      robot.clearBulkCache();
      telemetry.addData("AUTO", description);
      telemetry.addData("start", robot.follower.pose());
      telemetry.update();
    }
    return opMode.opModeIsActive();
  }

  // ---- poses -------------------------------------------------------------------------------------------------------

  /** A RED pose (x in, y in, heading deg) as this alliance's: BLUE's is rotated 180 degrees about the field centre. */
  protected Pose fieldPose(double x, double y, double redHeadingDeg) {
    double h = Math.toRadians(redHeadingDeg);
    if (robot.getAllianceColor() == AllianceColor.BLUE) {
      return new Pose(FIELD_IN - x, FIELD_IN - y, h + Math.PI);
    }
    return new Pose(x, y, h);
  }

  /** A RED pose given as {x, y} or {x, y, heading deg}. */
  protected Pose fieldPose(double[] red) {
    return fieldPose(red[0], red[1], red.length > 2 ? red[2] : 0);
  }

  /** The spot (a RED {x, y}) with the heading that puts the CELL in the turret's rest direction. */
  protected Pose shootingPose(double[] redSpot, Pose cell) {
    Pose p = fieldPose(redSpot);
    return p.withHeading(shootingHeading(p, cell));
  }

  /** Heading (rad) that puts the CELL in the turret's rest direction from (x, y): RED left (90), BLUE right (270). */
  protected double shootingHeading(Pose from, Pose cell) {
    double bearing = Math.atan2(cell.y() - from.y(), cell.x() - from.x());
    // turret position 180 = shooter forward, 90 = left, 270 = right: the rest direction is (180 - steadyState) CCW
    double rest = Math.toRadians(180 - robot.getAllianceColor().getSteadyState());
    return bearing - rest;
  }

  // ---- the loop ----------------------------------------------------------------------------------------------------

  /** The tags' tx (deg) at which the shot goes into the CELL from the next shooting spot. */
  protected void setAimPoint(double aimTxDeg) {
    this.aimTxDeg = aimTxDeg;
  }

  /** Done shooting: the turret goes back to its rest direction (the team's updateTarget without aim) and the flywheel stops. */
  protected void stopAiming() {
    aiming = false;
    robot.outtake.setTargetVelocity(0);
  }

  /**
   * One loop: flywheel target from the Limelight when it sees our upward CELL, else the turret holds its rest
   * direction (where the CELL will be when the robot reaches its shooting spot).
   */
  protected void update() {
    robot.clearBulkCache();
    if (aiming && robot.limelight.hasValidTarget()) {
      robot.outtake.setTargetVelocity(robot.limelight.calculateTargetVelocity());
    } else if (aiming) {
      robot.limelight.setTarget(robot.getAllianceColor().getSteadyState());
    }
    robot.follower.update();
    robot.outtake.updatePIDControl();
    aim();
    boolean ready = robot.limelight.hasValidTarget()
        && Math.abs(robot.limelight.error) < AIM_TOLERANCE_DEG
        && robot.outtake.atTarget(FLYWHEEL_TOLERANCE);
    if (ready && !readyNow) {
      readyFor.reset();
    }
    readyNow = ready;
    telemetry.addData("phase", phase);
    telemetry.addData("pose", robot.follower.pose());
    telemetry.addData("hive seen", robot.limelight.hasValidTarget());
    telemetry.addData("distance m", robot.limelight.distance);
    telemetry.addData("aim error", robot.limelight.error);
    telemetry.addData("Vel Target", robot.outtake.getTargetVelocity());
    telemetry.addData("Vel Current", robot.outtake.getCurrentVelocity());
    telemetry.update();
  }

  /**
   * The team's Limelight turret aim (Robot.updateAimingSystem) with this spot's aim point: once the CELL's tags are
   * seen the target is trimmed by (getAimPose - aimTxDeg); before that the turret holds the pre-aim set in update().
   */
  private void aim() {
    if (aiming) {
      Twist v = robot.follower.twist();
      robot.limelight.updateAim((-v.vy * 2.54) / 100.0, (v.vx * 2.54) / 100.0);
    }
    robot.limelight.updateTarget(robot.outtake.getTurretPosDegrees(), aiming, robot.getAllianceColor());
    if (aiming && robot.limelight.hasValidTarget()) {
      double trim = robot.getAllianceColor().getAimPose() - aimTxDeg;
      robot.limelight.target += trim;
      robot.limelight.error += trim;
    }
    robot.outtake.setPowerTurret(robot.limelight.updateAimPID());
  }

  // ---- steps -------------------------------------------------------------------------------------------------------

  /**
   * Follows the path until it ends or {@code timeoutMs} passes, then holds its end pose (a pose against a wall or a
   * flower keeps the robot pressed there). False on a timeout.
   */
  protected boolean drive(Path path, int timeoutMs) {
    robot.follower.follow(path);
    ElapsedTime timer = new ElapsedTime();
    while (opMode.opModeIsActive() && robot.follower.isBusy() && timer.milliseconds() < timeoutMs) {
      update();
    }
    boolean done = !robot.follower.isBusy();
    robot.follower.hold(path.endPose());
    return done;
  }

  /**
   * {@link #drive} onto a spot that has to be hit (a shooting spot): if Pedro stopped more than SETTLE_IN off the
   * path's end (after a long or curved path the untuned follower ended 2-3 in off in the simulator), a short straight
   * path onto it.
   */
  protected void driveTo(Path path, int timeoutMs) {
    drive(path, timeoutMs);
    Pose end = path.endPose();
    Pose p = robot.follower.pose();
    if (opMode.opModeIsActive() && Math.hypot(end.x() - p.x(), end.y() - p.y()) > SETTLE_IN) {
      drive(Paths.line(p, end).linear(p, end), SETTLE_TIMEOUT_MS);
    }
  }

  protected void waitMs(double ms) {
    ElapsedTime timer = new ElapsedTime();
    while (opMode.opModeIsActive() && timer.milliseconds() < ms) {
      update();
    }
  }

  /** Waits for the turret on the tags' aim point and the flywheel at speed; false when it gave up after {@code timeoutMs}. */
  protected boolean aimUntilReady(int timeoutMs) {
    ElapsedTime timer = new ElapsedTime();
    while (opMode.opModeIsActive() && timer.milliseconds() < timeoutMs && !readyToShoot()) {
      update();
    }
    return readyToShoot();
  }

  protected boolean readyToShoot() {
    return readyNow && readyFor.milliseconds() >= READY_HOLD_MS;
  }

  /**
   * Shoots {@code balls} balls one at a time, each once the aim is ready again (or after SHOT_READY_TIMEOUT_MS with
   * the CELL still in view), and keeps the rest: the intake runs just long enough for one ball to reach the flywheel.
   * Stops early when the Limelight loses our upward CELL (never shoot at a CELL that is not up, G417). Returns the
   * number fed.
   */
  protected int shoot(int balls) {
    openGate();
    waitMs(GATE_OPEN_MS);
    int fed = 0;
    while (fed < balls && opMode.opModeIsActive()) {
      if (!aimUntilReady(SHOT_READY_TIMEOUT_MS) && !robot.limelight.hasValidTarget()) {
        break;
      }
      robot.intake.setIntakePower(INTAKE_POWER);
      waitMs(FEED_FIRST_MS + FEED_MARGIN_MS);
      robot.intake.setIntakePower(0);
      fed++;
    }
    closeGate();
    return fed;
  }

  /** Shoots the whole hopper in one burst: the intake runs SHOOT_ALL_MS with the gate open. */
  protected void shootVolley() {
    openGate();
    waitMs(GATE_OPEN_MS);
    robot.intake.setIntakePower(INTAKE_POWER);
    waitMs(SHOOT_ALL_MS);
    robot.intake.setIntakePower(0);
    closeGate();
  }

  /** Runs the intake with the gate shut (collecting into the hopper) or stops it. */
  protected void intake(boolean on) {
    robot.intake.setIntakePower(on ? INTAKE_POWER : 0);
  }

  protected void openGate() {
    robot.intake.setFlapper(true);
  }

  protected void closeGate() {
    robot.intake.setFlapper(false);
  }

  /** Holds the last pose until AUTO ends. */
  protected void holdUntilEnd() {
    while (opMode.opModeIsActive()) {
      update();
    }
  }
}
