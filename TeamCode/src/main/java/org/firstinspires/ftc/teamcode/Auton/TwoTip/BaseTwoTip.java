package org.firstinspires.ftc.teamcode.Auton.TwoTip;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auton.BaseHiveAuto;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Two HIVE tips in AUTO (drafted in the simulator).
 *
 * <ol>
 *   <li>Out from the audience wall, line up in front of the audience CELL (up at the start, holding its 3 NECTAR) and
 *       shoot all 4 preloads in one volley: the third POLLEN tips the HIVE.</li>
 *   <li>Sweep the GARDEN line parallel to the audience wall with the intake running and the gate shut (the flower arm
 *       stays up): POLLEN fill the hopper.</li>
 *   <li>Behind the HIVE: the tip turned the scoring CELL up, opening to the rear. Shoot the 4.</li>
 *   <li>The rear flower, flower arm down: its 4 POLLEN through the bottom retrieval opening.</li>
 *   <li>Back to the same spot, shoot the 4: 8 POLLEN tip the HIVE again.</li>
 *   <li>Park in the LOADING ZONE, clear of the wall (LEAVE and PARK are judged when AUTO ends).</li>
 * </ol>
 *
 * Only POLLEN fits the intake (NECTAR is 3.6 in), and the HIVE tips at 8 POLLEN or 3 NECTAR + 3 POLLEN, so the empty
 * scoring CELL needs 8.
 */
@Configurable
public class BaseTwoTip extends BaseHiveAuto {

  /** RED: straight out from the audience wall first, x in, y in. */
  public static double[] OUT_RED = {46.0, 22.0};
  /** RED spot for the audience CELL, x in, y in: 1.43 m from its opening, 47 deg off its axis. */
  public static double[] SHOOT_A_RED = {16.5, 16.5};
  /** RED audience CELL opening, x in, y in (up at the start). */
  public static double[] CELL_A_RED = {57.8, 54.8};

  /**
   * RED GARDEN sweep, x in, y in, heading deg (tuned on the robot, 2026-11-13): the whole turn to 180 happens at
   * GARDEN_TURN as its own path, then a slide down to GARDEN_WALL and a slow sweep toward the red wall at y 12 (the
   * robot's left side, 8.37 in from its centre, stays ~3.6 in off the audience wall).
   */
  public static double[] GARDEN_TURN_RED = {28.0, 18.0, 180};
  public static double[] GARDEN_WALL_RED = {25.0, 12, 180};
  public static double[] GARDEN_END_RED = {11.3, 12, 180};
  public static int WALL_TIMEOUT_MS = 1500;
  public static int SWEEP_TIMEOUT_MS = 1500;
  /** Path speed limit while sweeping the GARDEN and closing in on the flower (in/s). */
  public static double SLOW_SPEED = 25;
  /** The flower arm's travel down before the rear-flower pickup starts (ms). */
  public static int FLOWER_ARM_MS = 250;

  /** RED curve from the GARDEN to SHOOT_B (control points, x in, y in): clear of the red-wall flower and the HIVE. */
  public static double[] VIA_1_RED = {26.0, 40.0};
  public static double[] VIA_2_RED = {26.0, 95.0};
  /**
   * RED spot for the scoring CELL, x in, y in: 1.47 m from its opening, 47 deg off its axis (closer than ~1.4 m the
   * formula's speed climbs and POLLEN bounce back out of the CELL). With the CELL on the turret's right the robot's
   * corners are 4.4 in off the red wall and 2.2 in off the rear wall.
   */
  public static double[] SHOOT_B_RED = {15.5, 125.9};
  /** RED scoring CELL opening, x in, y in (up after the first tip, facing the rear). */
  public static double[] CELL_B_RED = {57.8, 86.25};

  /**
   * RED rear flower (column x 47.13, plate face 136.16 in): in front of it, then the pickup with the front ~0.3 in short
   * of the plate. x in, y in, heading deg (tuned on the robot, 2026-11-13). At x 42.5 the intake roller spans x
   * 36.1-48.2, covering the bottom retrieval opening (x 45.4-48.9) up to its last 0.7 in.
   */
  public static double[] FLOWER_APPROACH_RED = {42.5, 114.0, 90};
  public static double[] FLOWER_RED = {42.5, 126, 90};
  /** Intake running at the flower: 4 POLLEN, one per feed interval as the stack drops (ms). */
  public static int RETRIEVE_MS = 1000;

  /** RED park, x in, y in, heading deg: the robot's left side 2.8 in into the LOADING ZONE, clear of the wall. */
  public static double[] PARK_RED = {17.0, 106.0, 90};

  public static int DRIVE_TIMEOUT_MS = 5000;
  public static int PRESS_TIMEOUT_MS = 1500;

  public BaseTwoTip(LinearOpMode opMode, Robot robot) {
    super(opMode, robot);
  }

  @Override
  public void run() {
    if (!initAndWaitForStart("2 TIP: 4 preloads, garden + rear flower into the scoring CELL")) {
      return;
    }
    Pose cellA = fieldPose(CELL_A_RED);
    Pose cellB = fieldPose(CELL_B_RED);
    Pose start = robot.follower.pose();
    Pose out = fieldPose(OUT_RED).withHeading(start.heading());
    Pose shootA = shootingPose(SHOOT_A_RED, cellA);
    Pose gardenTurn = fieldPose(GARDEN_TURN_RED);
    Pose gardenWall = fieldPose(GARDEN_WALL_RED);
    Pose gardenEnd = fieldPose(GARDEN_END_RED);
    Pose shootB = shootingPose(SHOOT_B_RED, cellB);
    Pose flowerApproach = fieldPose(FLOWER_APPROACH_RED);
    Pose flower = fieldPose(FLOWER_RED);

    // --- 1. out, line up at the audience CELL, shoot all 4: the third tips the HIVE ---
    phase = "DRIVE A";
    robot.outtake.setTargetVelocity(SPINUP_TICKS);
    driveTo(Paths.path(Paths.line(start, out).constant(start.heading()), Paths.line(out, shootA).linear(out, shootA)),
        DRIVE_TIMEOUT_MS);
    phase = "AIM A";
    aimUntilReady(AIM_TIMEOUT_MS);
    phase = "SHOOT 4";
    shootVolley();

    // --- 2. the GARDEN: POLLEN fill the hopper (the flower arm stays up) ---
    phase = "GARDEN";
    driveTo(Paths.line(shootA, gardenTurn).linear(shootA, gardenTurn), DRIVE_TIMEOUT_MS);
    drive(Paths.line(gardenTurn, gardenWall).constant(gardenTurn.heading()), WALL_TIMEOUT_MS);
    intake(true);
    Pose atWall = robot.follower.pose();
    drive(Paths.line(atWall, gardenEnd).constant(gardenEnd.heading()).with(slow()), SWEEP_TIMEOUT_MS);

    // --- 3. behind the HIVE: shoot 4 into the scoring CELL ---
    phase = "DRIVE B";
    Pose atGarden = robot.follower.pose();   // where the sweep really ended, not exactly GARDEN_END
    driveTo(Paths.curve(atGarden, fieldPose(VIA_1_RED), fieldPose(VIA_2_RED), shootB).linear(atGarden, shootB),
        DRIVE_TIMEOUT_MS);
    intake(false);
    phase = "AIM B";
    if (aimUntilReady(AIM_TIMEOUT_MS) || robot.limelight.hasValidTarget()) {
      phase = "SHOOT B 1";
      shootVolley();
    }

    // --- 4. the rear flower's 4 POLLEN (flower arm down) ---
    phase = "FLOWER";
    drive(Paths.line(shootB, flowerApproach).linear(shootB, flowerApproach), DRIVE_TIMEOUT_MS);
    flowerArm(true);
    intake(true);
    waitMs(FLOWER_ARM_MS);
    drive(Paths.line(flowerApproach, flower).constant(flower.heading()).with(slow()), PRESS_TIMEOUT_MS);
    waitMs(RETRIEVE_MS);
    intake(false);
    flowerArm(false);

    // --- 5. back to SHOOT_B, shoot 4: 8 POLLEN tip the HIVE again ---
    phase = "DRIVE B";
    Pose atFlower = robot.follower.pose();
    driveTo(Paths.line(atFlower, shootB).linear(atFlower, shootB), DRIVE_TIMEOUT_MS);
    phase = "AIM B";
    if (aimUntilReady(AIM_TIMEOUT_MS) || robot.limelight.hasValidTarget()) {
      phase = "SHOOT B 2";
      shootVolley();
    }

    // --- 6. park ---
    phase = "PARK";
    stopAiming();
    Pose park = fieldPose(PARK_RED);
    driveTo(Paths.line(shootB, park).linear(shootB, park), DRIVE_TIMEOUT_MS);
    phase = "PARKED";
    holdUntilEnd();
  }

  /** Speed limit for the GARDEN sweep and the flower press (the team's Foresight config, only during that path). */
  private static com.pedropathing.config.Modifier slow() {
    return Constants.foresightConfig.maxPathSpeed.at(SLOW_SPEED);
  }
}
