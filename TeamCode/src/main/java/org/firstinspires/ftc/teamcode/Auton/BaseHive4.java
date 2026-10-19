package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
  /** RED shooting spot, x in, y in: ~1.43 m from the CELL opening, 47 deg off its axis, clear of the garden pollen. */
  public static double[] SHOOT_RED = {16.5, 16.5};
  /** RED upward CELL opening (the audience CELL at the start), x in, y in. */
  public static double[] CELL_RED = {57.8, 54.8};

  /**
   * The tags' mean tx (deg) at which the shooter points at the CELL opening from the shooting spot: the camera sits
   * 82 mm right of the shot line and, 47 deg off the CELL's axis, the tag cluster (73 mm deeper than the opening)
   * appears beside it. In the simulator it is -4.5 for both alliances; the teleop's AllianceColor.getAimPose() is -5 (RED) /
   * -1 (BLUE), so this auto trims the team's Limelight aim by (getAimPose - AIM_TX_DEG). Tune on the robot from the
   * first shot's miss.
   */
  public static double AIM_TX_DEG = -4.5;

  public BaseHive4(LinearOpMode opMode, Robot robot) {
    super(opMode, robot);
  }

  @Override
  public void run() {
    if (!initAndWaitForStart("HIVE 4: out, line up, shoot 4")) {
      return;
    }
    Pose cell = fieldPose(CELL_RED);
    setAimPoint(AIM_TX_DEG);

    // --- drive out and line up ---
    phase = "DRIVE";
    Pose start = robot.follower.pose();
    Pose out = fieldPose(OUT_RED).withHeading(start.heading());
    Pose shoot = shootingPose(SHOOT_RED, cell);
    robot.outtake.setTargetVelocity(SPINUP_TICKS);
    drive(Paths.path(Paths.line(start, out).constant(start.heading()), Paths.line(out, shoot).linear(out, shoot)), 6000);

    // --- aim and spin up ---
    phase = "AIM";
    aimUntilReady(AIM_TIMEOUT_MS);

    // --- shoot the 4 preloads ---
    phase = "SHOOT";
    shootVolley();

    // --- hold the shooting spot until AUTO ends ---
    phase = "DONE";
    holdUntilEnd();
  }
}
