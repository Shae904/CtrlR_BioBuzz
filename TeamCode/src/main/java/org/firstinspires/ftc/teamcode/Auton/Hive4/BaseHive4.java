package org.firstinspires.ftc.teamcode.Auton.Hive4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Auton.BaseHiveAuto;
import org.firstinspires.ftc.teamcode.Auton.TwoTip.BaseTwoTip;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * First BIOBUZZ autonomous (drafted in the simulator): drive out from the start pose, line up in front of our HIVE's
 * upward CELL, aim the turret on the CELL's AprilTags and shoot the 4 preloads through the gate. The CELL starts with 3
 * NECTAR, so the third POLLEN tips the HIVE and the fourth spills with it ({@link BaseTwoTip} keeps it instead).
 */
@Configurable
public class BaseHive4 extends BaseHiveAuto {

  /** RED: straight out from the audience wall first (clear of the wall before turning), x in, y in. */
  public static double[] OUT_RED = {46.0, 22.0};
  /**
   * RED shooting spot, x in, y in: 1.44 m from the CELL opening, clear of the garden pollen. 13.2 (was 12, comp 1):
   * with the turret on the robot's right (both alliances since 2026-11-14) the robot's front-left corner is 0.5 in off
   * the red wall here; at 12 it was 0.64 in into it. EXP_BLUE HIVE 4 uses this spot too (rotated).
   */
  public static double[] SHOOT_RED = {13.2, 20};
  /**
   * BLUE HIVE 4's shooting spot, written in RED coordinates (rotated for BLUE like every pose): the comp 1 spot, kept
   * exactly as it was. Here BLUE's robot has its front-left corner 0.64 in into the blue wall; EXP_BLUE HIVE 4 tries
   * SHOOT_RED's 13.2 instead.
   */
  public static double[] SHOOT_BLUE_AS_RED = {12, 20};
  /** RED upward CELL opening (the audience CELL at the start), x in, y in. */
  public static double[] CELL_RED = {57.8, 54.8};

  public static double[] PARK_RED = {14.0, 106.0, 90};
  /**
   * RED control point of the curve from the shooting spot to the park, x in, y in: bows the path away from the red
   * wall so the turning robot clears the red-wall flower (plate x 0-4.9, y 44.2-50.1) by ~8.4 in and the HIVE's west
   * foot bar (x 45.8, y 51-90) by ~9.0 in. The straight line passed the flower with 0.05 in to spare.
   */
  public static double[] PARK_VIA_RED = {34.0, 64.0};

  /** EXP_BLUE HIVE 4: BLUE on SHOOT_RED's spot instead of SHOOT_BLUE_AS_RED. */
  private final boolean experimental;

  public BaseHive4(LinearOpMode opMode, Robot robot) {
    this(opMode, robot, false);
  }

  public BaseHive4(LinearOpMode opMode, Robot robot, boolean experimental) {
    super(opMode, robot);
    this.experimental = experimental;
  }

  /** The shooting spot in RED coordinates: BLUE keeps its comp 1 spot unless experimental. */
  private double[] shootSpot() {
    return robot.getAllianceColor() == AllianceColor.BLUE && !experimental ? SHOOT_BLUE_AS_RED : SHOOT_RED;
  }

  @Override
  public void run() {
    if (!initAndWaitForStart((experimental ? "EXPERIMENTAL " : "") + "HIVE 4: out, line up, shoot 4")) {
      return;
    }
    Pose cell = fieldPose(CELL_RED);

    // --- drive out and line up ---
    phase = "DRIVE";
    Pose start = robot.follower.pose();
    Pose out = fieldPose(OUT_RED).withHeading(start.heading());
    Pose shoot = shootingPose(shootSpot(), cell);
    robot.outtake.setTargetVelocity(SPINUP_TICKS);
    drive(Paths.path(Paths.line(start, out).constant(start.heading()), Paths.line(out, shoot).linear(out, shoot)), 6000);

    // --- aim and spin up ---
    phase = "AIM";
    aimUntilReady(AIM_TIMEOUT_MS);

    // --- shoot the 4 preloads ---
    phase = "SHOOT";
    shootVolley();

    // --- hold the shooting spot until AUTO ends ---
    phase = "PARK";
    stopAiming();
    Pose park = fieldPose(PARK_RED);
    driveTo(Paths.curve(shoot, fieldPose(PARK_VIA_RED), park).linear(shoot, park), 6000);
    phase = "PARKED";
    holdUntilEnd();
  }
}
