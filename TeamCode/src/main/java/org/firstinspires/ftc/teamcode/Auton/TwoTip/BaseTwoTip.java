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
 *       shoot 3 preloads: the third POLLEN tips the HIVE. The fourth preload stays in the hopper.</li>
 *   <li>Sweep the GARDEN line along the audience wall with the flower arm down, the intake running and the gate shut:
 *       3 more POLLEN fill the hopper.</li>
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
   * RED GARDEN sweep, x in, y in, heading deg (the robot's left side, its wheels, is 8.37 in from its centre):
   * <ul>
   *   <li>GARDEN_TURN: the whole turn to 180 happens here, 9.6 in clear of the audience wall, as its own path (it was
   *       folded into the slide to the wall, so the robot could reach the wall still turning, at an angle).</li>
   *   <li>GARDEN_WALL: slide straight down beside the wall, 0.2 in off it, so Pedro can reach the end (at 0.03 in, any
   *       odometry error hit the wall first and the drive waited out its timeout pushing on it).</li>
   *   <li>GARDEN_END: sweep toward the red wall, closing the last 0.2 in and riding the wall with a light 0.27 in press
   *       (it was 1.37 in into the wall, so the follower strafed the left wheels into it for the whole sweep and kept
   *       pushing after). It ends with the front against the corner POLLEN.</li>
   * </ul>
   */
  public static double[] GARDEN_TURN_RED = {28.0, 18.0, 190};
  public static double[] GARDEN_WALL_RED = {25.0, 11, 190};
  public static double[] GARDEN_END_RED = {12.3, 10.1, 190};
  public static int WALL_TIMEOUT_MS = 1500;
  public static int SWEEP_TIMEOUT_MS = 1500;
  /** Path speed limit while sweeping the GARDEN and closing in on the flower (in/s). */
  public static double SLOW_SPEED = 25;
  /** The flower arm's travel down before a pickup starts (ms). */
  public static int FLOWER_ARM_MS = 250;

  /** RED curve from the GARDEN to SHOOT_B (control points, x in, y in): clear of the red-wall flower and the HIVE. */
  public static double[] VIA_1_RED = {26.0, 40.0};
  public static double[] VIA_2_RED = {26.0, 95.0};
  /**
   * RED spot for the scoring CELL, x in, y in: 1.47 m from its opening, 47 deg off its axis (closer than ~1.4 m the
   * formula's speed climbs and POLLEN bounce back out of the CELL), with the robot's corners 1.8 in off the red wall
   * and 2.2 in off the rear wall (BLUE's robot faces the other way: the CELL is on its turret's right).
   */
  public static double[] SHOOT_B_RED = {15.5, 125.9};
  /** RED scoring CELL opening, x in, y in (up after the first tip, facing the rear). */
  public static double[] CELL_B_RED = {57.8, 86.25};

  /**
   * RED rear flower (column x 47.13, plate face 4.89 in off the rear wall): in front of it, then the front 0.1 in short
   * of the plate, where the intake reaches 1.7 in into the bottom retrieval opening. x in, y in, heading deg. x 50.5 =
   * 3 in to the robot's right of the column line (was 47.5; on the robot it did not pick the stack up properly).
   */
  public static double[] FLOWER_APPROACH_RED = {50.5, 117.0, 90};
  public static double[] FLOWER_RED = {48.5, 134.2, 90};
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
    if (!initAndWaitForStart("2 TIP: 3 preloads, garden + rear flower into the scoring CELL")) {
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

    // --- 1. out, line up at the audience CELL, shoot 3: the third tips the HIVE ---
    phase = "DRIVE A";
    robot.outtake.setTargetVelocity(SPINUP_TICKS);
    driveTo(Paths.path(Paths.line(start, out).constant(start.heading()), Paths.line(out, shootA).linear(out, shootA)),
        DRIVE_TIMEOUT_MS);
    phase = "AIM A";
    aimUntilReady(AIM_TIMEOUT_MS);
    phase = "SHOOT 4";
    shootVolley();

    // --- 2. the GARDEN: 3 POLLEN fill the hopper (flower arm down) ---
    phase = "GARDEN";
    driveTo(Paths.line(shootA, gardenTurn).linear(shootA, gardenTurn), DRIVE_TIMEOUT_MS);
    drive(Paths.line(gardenTurn, gardenWall).constant(gardenTurn.heading()), WALL_TIMEOUT_MS);
    intake(true);
    waitMs(FLOWER_ARM_MS);
    Pose atWall = robot.follower.pose();
    drive(Paths.line(atWall, gardenEnd).constant(gardenEnd.heading()).with(slow()), SWEEP_TIMEOUT_MS);
    flowerArm(false);

    // --- 3. behind the HIVE: shoot 4 into the scoring CELL ---
    phase = "DRIVE B";
    Pose atGarden = robot.follower.pose();   // pressed on the wall, not exactly at GARDEN_END
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
