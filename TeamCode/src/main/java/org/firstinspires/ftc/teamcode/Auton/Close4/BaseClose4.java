package org.firstinspires.ftc.teamcode.Auton.Close4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.BaseHiveAuto;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * CLOSE 4: starts beside our LOADING ZONE, waits for our HIVE's CELL on this side of the field to come up (an
 * alliance partner's tip turns it up), shoots the 4 preloads into it, and parks in the LOADING ZONE.
 *
 * <ol>
 *   <li>From the start corner to the shooting spot beside the LOADING ZONE (2 TIP's scoring-CELL spot), the CELL on
 *       the turret's rest side (the robot's right), flywheel spinning.</li>
 *   <li>Wait until the Limelight sees that CELL up (it only counts a CELL whose tags are at the up height, so a CELL
 *       that is still down never counts), at most until SEE_DEADLINE_S after START.</li>
 *   <li>Seen: aim, shoot all 4, park. Not seen by the deadline: park without shooting.</li>
 * </ol>
 *
 * RED waits for the CELL the code calls RED SCORING (tags 30-33, facing the rear wall); BLUE, rotated 180 degrees,
 * for BLUE AUDIENCE (tags 38-41, facing the audience wall). Both are the CELL that is down at the start and faces this
 * robot's side. Volley and park take ~5 s, so a CELL seen after ~25 s still gets shot at, but the robot then reaches the
 * LOADING ZONE after AUTO ends (lower SEE_DEADLINE_S to favour PARK).
 */
@Configurable
public class BaseClose4 extends BaseHiveAuto {

  /** RED: out of the start corner before turning (clear of the rear wall), x in, y in. */
  public static double[] OUT_RED = {13.0, 127.5};
  /** RED shooting spot beside the LOADING ZONE, x in, y in: 1.47 m from the CELL opening (same as 2 TIP's SHOOT_B). */
  public static double[] SHOOT_RED = {15.5, 125.9};
  /** RED CELL opening on this side (RED SCORING, up after a tip, facing the rear wall), x in, y in. */
  public static double[] CELL_RED = {57.8, 86.25};
  /** RED park, x in, y in, heading deg: the robot's left side 2.8 in into the LOADING ZONE (same as 2 TIP's). */
  public static double[] PARK_RED = {17.0, 106.0, 90};
  /** Stop waiting for the CELL this long after START (s) and just park. */
  public static double SEE_DEADLINE_S = 27.0;
  public static int DRIVE_TIMEOUT_MS = 4000;

  public BaseClose4(LinearOpMode opMode, Robot robot) {
    super(opMode, robot);
  }

  @Override
  public void run() {
    if (!initAndWaitForStart("CLOSE 4: wait for our CELL (up to " + SEE_DEADLINE_S + " s), shoot 4, park")) {
      return;
    }
    ElapsedTime sinceStart = new ElapsedTime();
    Pose cell = fieldPose(CELL_RED);
    Pose start = robot.follower.pose();
    Pose out = fieldPose(OUT_RED).withHeading(start.heading());
    Pose shoot = shootingPose(SHOOT_RED, cell);

    // --- 1. to the shooting spot, flywheel spinning ---
    phase = "DRIVE";
    robot.outtake.setTargetVelocity(SPINUP_TICKS);
    driveTo(Paths.path(Paths.line(start, out).constant(start.heading()), Paths.line(out, shoot).linear(out, shoot)),
        DRIVE_TIMEOUT_MS);

    // --- 2. wait for the CELL to be up and in view, then shoot all 4 ---
    boolean shot = false;
    while (opMode.opModeIsActive() && !shot && sinceStart.seconds() < SEE_DEADLINE_S) {
      phase = String.format("WAIT FOR CELL (%.0f s left)", SEE_DEADLINE_S - sinceStart.seconds());
      update();
      if (robot.limelight.hasValidTarget()) {
        phase = "AIM";
        if (aimUntilReady(AIM_TIMEOUT_MS) || robot.limelight.hasValidTarget()) {
          phase = "SHOOT";
          shootVolley();
          shot = true;
        }
      }
    }

    // --- 3. park ---
    phase = shot ? "PARK" : "PARK (CELL not seen)";
    stopAiming();
    Pose here = robot.follower.pose();
    Pose park = fieldPose(PARK_RED);
    driveTo(Paths.line(here, park).linear(here, park), DRIVE_TIMEOUT_MS);
    phase = "PARKED";
    holdUntilEnd();
  }
}
