package org.firstinspires.ftc.teamcode.Auton.TwoTip;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auton.BaseHiveAuto;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Two HIVE tips in AUTO, head-on (2026-11-14; the previous side-spot route is still {@code RED/BLUE 2 TIP OLD}).
 *
 * <ol>
 *   <li>From the start pose (against the audience wall, facing the HIVE), shoot all 4 preloads at the audience CELL
 *       (up at the start, holding its 3 NECTAR): the third POLLEN tips the HIVE. The turret pre-aims at the CELL from
 *       odometry (it is nearly straight ahead, not in the turret's rest direction) until the camera has the tags.</li>
 *   <li>Sweep the GARDEN line parallel to the audience wall with the intake running and the gate shut (the flower arm
 *       stays up): POLLEN fill the hopper.</li>
 *   <li>Through the middle of the HIVE structure (under it, between its floor bars) to spot B: the start pose mirrored
 *       through the HIVE's pivot, so the scoring CELL (up after the tip, opening to the rear) is at exactly the
 *       distance and angle the audience CELL was from the start. Shoot the 4.</li>
 *   <li>The rear flower, flower arm down: its POLLEN through the bottom retrieval opening.</li>
 *   <li>Back to spot B, shoot: 8 POLLEN tip the HIVE again.</li>
 *   <li>Park in the LOADING ZONE (LEAVE and PARK are judged when AUTO ends).</li>
 * </ol>
 *
 * Only POLLEN fits the intake (NECTAR is 3.6 in), and the HIVE tips at 8 POLLEN or 3 NECTAR + 3 POLLEN, so the empty
 * scoring CELL needs 8.
 */
@Configurable
public class BaseTwoTip extends BaseHiveAuto {

  /** RED: straight out from the audience wall first, x in, y in. */
  public static double[] OUT_RED = {46.0, 22.0};
  /** RED audience CELL opening, x in, y in (up at the start). */
  public static double[] CELL_A_RED = {57.8, 54.8};
  /** RED scoring CELL opening, x in, y in (up after the first tip, facing the rear). */
  public static double[] CELL_B_RED = {57.8, 86.25};
  /**
   * RED HIVE pivot, x in, y in (field CAD): spot B is the start pose mirrored through it (heading + 180), so from B the
   * scoring CELL is where the audience CELL was from the start. From the RED start (46, 7.36, 90) that is
   * (69.57, 133.69, 270): back against the rear wall, 9.9 in clear of the rear flower, 1.24 m from the opening.
   */
  public static double[] PIVOT_RED = {57.783, 70.524};
  /** From spot B (against the rear wall) this far straight toward the HIVE before turning (in). */
  public static double B_PULL_OUT_IN = 9.7;
  /** The first volley starts with the flywheel at rest (no drive to spin up during), so it may wait longer (ms). */
  public static int AIM_A_TIMEOUT_MS = 2500;

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

  /**
   * RED curve from the GARDEN to spot B through the middle of the HIVE structure (control points, x in, y in): out
   * along the audience side, then up the open lane under the structure. Robot outline swept along it: 8.7 in from the
   * west floor bar, 10.6 in from the east one, 6.3 in from the nearest A-frame leg (at the robot's 17.8 in height).
   */
  public static double[] CENTER_VIA_1_RED = {80.0, 10.0};
  public static double[] CENTER_VIA_2_RED = {74.0, 55.0};
  /** The curve through the middle is ~150 in long (ms). */
  public static int CENTER_TIMEOUT_MS = 7000;

  /**
   * RED rear flower (column x 47.13, plate face 136.16 in): in front of it, then the pickup. x in, y in, heading deg
   * (tuned on the robot). At x 42.5 the intake roller spans x 36.1-48.2, covering the bottom retrieval opening
   * (x 45.4-48.9) up to its last 0.7 in.
   */
  public static double[] FLOWER_APPROACH_RED = {42.5, 114.0, 90};
  public static double[] FLOWER_RED = {42.5, 126.7, 90};
  /** Intake running at the flower: 4 POLLEN, one per feed interval as the stack drops (ms). */
  public static int RETRIEVE_MS = 3500;

  /** RED park, x in, y in, heading deg: the robot's left side 2.8 in into the LOADING ZONE, clear of the wall. */
  public static double[] PARK_RED = {17.0, 106.0, 90};

  public static int DRIVE_TIMEOUT_MS = 5000;
  public static int PRESS_TIMEOUT_MS = 1500;

  // ---- the previous route (RED/BLUE 2 TIP OLD): side spots, around the outside of the HIVE ----
  /** OLD: RED spot for the audience CELL, x in, y in: 1.43 m from its opening, 47 deg off its axis. */
  public static double[] OLD_SHOOT_A_RED = {16.5, 16.5};
  /** OLD: RED curve from the GARDEN to OLD_SHOOT_B along the red wall (control points, x in, y in). */
  public static double[] OLD_VIA_1_RED = {26.0, 40.0};
  public static double[] OLD_VIA_2_RED = {26.0, 95.0};
  /** OLD: RED spot for the scoring CELL, x in, y in: 1.47 m from its opening, 47 deg off its axis. */
  public static double[] OLD_SHOOT_B_RED = {15.5, 125.9};

  private final boolean oldRoute;

  public BaseTwoTip(LinearOpMode opMode, Robot robot) {
    this(opMode, robot, false);
  }

  /** @param oldRoute the previous route (side spots, around the outside of the HIVE) instead of the head-on one */
  public BaseTwoTip(LinearOpMode opMode, Robot robot, boolean oldRoute) {
    super(opMode, robot);
    this.oldRoute = oldRoute;
  }

  @Override
  public void run() {
    if (oldRoute) {
      runOldRoute();
      return;
    }
    if (!initAndWaitForStart("2 TIP head-on: shoot 4 from the start, garden, under the HIVE, rear flower")) {
      return;
    }
    Pose cellA = fieldPose(CELL_A_RED);
    Pose cellB = fieldPose(CELL_B_RED);
    Pose start = robot.follower.pose();
    Pose pivot = fieldPose(PIVOT_RED);
    Pose shootB = new Pose(2 * pivot.x() - start.x(), 2 * pivot.y() - start.y(), start.heading() + Math.PI);
    Pose bOut = new Pose(shootB.x() + Math.cos(shootB.heading()) * B_PULL_OUT_IN,
        shootB.y() + Math.sin(shootB.heading()) * B_PULL_OUT_IN, shootB.heading());
    Pose out = fieldPose(OUT_RED).withHeading(start.heading());
    Pose gardenTurn = fieldPose(GARDEN_TURN_RED);
    Pose gardenWall = fieldPose(GARDEN_WALL_RED);
    Pose gardenEnd = fieldPose(GARDEN_END_RED);
    Pose flowerApproach = fieldPose(FLOWER_APPROACH_RED);
    Pose flower = fieldPose(FLOWER_RED);

    // --- 1. from the start pose, shoot all 4 at the audience CELL: the third tips the HIVE ---
    phase = "AIM A";
    robot.intake.setFlower(false);
    robot.outtake.setTargetVelocity(SPINUP_TICKS);
    preAim(cellA);
    aimUntilReady(AIM_A_TIMEOUT_MS);
    phase = "SHOOT A";
    shootVolley();
    preAim(null);

    // --- 2. the GARDEN: POLLEN fill the hopper (the flower arm stays up) ---
    phase = "GARDEN";
    driveTo(Paths.path(Paths.line(start, out).constant(start.heading()), Paths.line(out, gardenTurn).linear(out, gardenTurn)),
        DRIVE_TIMEOUT_MS);
    drive(Paths.line(gardenTurn, gardenWall).constant(gardenTurn.heading()), WALL_TIMEOUT_MS);
    intake(true);
    Pose atWall = robot.follower.pose();
    drive(Paths.line(atWall, gardenEnd).constant(gardenEnd.heading()).with(slow()), SWEEP_TIMEOUT_MS);

    // --- 3. through the middle of the HIVE structure to spot B, shoot 4 into the scoring CELL ---
    phase = "DRIVE B";
    Pose atGarden = robot.follower.pose();   // where the sweep really ended, not exactly GARDEN_END
    driveTo(Paths.curve(atGarden, fieldPose(CENTER_VIA_1_RED), fieldPose(CENTER_VIA_2_RED), shootB)
        .linear(atGarden, shootB), CENTER_TIMEOUT_MS);
    intake(false);
    shootAtB("SHOOT B 1", cellB);

    // --- 4. the rear flower's POLLEN (flower arm down) ---
    phase = "FLOWER";
    drive(Paths.path(Paths.line(shootB, bOut).constant(shootB.heading()),
        Paths.line(bOut, flowerApproach).linear(bOut, flowerApproach)), DRIVE_TIMEOUT_MS);
    flowerArm(true);
    intake(true);
    waitMs(FLOWER_ARM_MS);
    drive(Paths.line(flowerApproach, flower).constant(flower.heading()).with(slow()), PRESS_TIMEOUT_MS);
    waitMs(RETRIEVE_MS);
    intake(false);
    flowerArm(false);

    // --- 5. back to spot B, shoot: 8 POLLEN tip the HIVE again ---
    phase = "DRIVE B";
    Pose atFlower = robot.follower.pose();
    driveTo(Paths.path(Paths.line(atFlower, flowerApproach).constant(flower.heading()),
        Paths.line(flowerApproach, bOut).linear(flowerApproach, bOut),
        Paths.line(bOut, shootB).constant(shootB.heading())), DRIVE_TIMEOUT_MS);
    shootAtB("SHOOT B 2", cellB);

    // --- 6. park ---
    phase = "PARK";
    stopAiming();
    Pose park = fieldPose(PARK_RED);
    driveTo(Paths.path(Paths.line(shootB, bOut).constant(shootB.heading()), Paths.line(bOut, park).linear(bOut, park)),
        DRIVE_TIMEOUT_MS);
    phase = "PARKED";
    holdUntilEnd();
  }

  /** At spot B: pre-aim at the scoring CELL, then shoot only once its tags confirm it is up. */
  private void shootAtB(String shootPhase, Pose cellB) {
    phase = "AIM B";
    preAim(cellB);
    if (aimUntilReady(AIM_TIMEOUT_MS) || robot.limelight.hasValidTarget()) {
      phase = shootPhase;
      shootVolley();
    }
    preAim(null);
  }

  /** The previous route, unchanged: side spots 47 deg off each CELL's axis, around the outside of the HIVE. */
  private void runOldRoute() {
    if (!initAndWaitForStart("2 TIP OLD: 4 preloads, garden + rear flower into the scoring CELL")) {
      return;
    }
    Pose cellA = fieldPose(CELL_A_RED);
    Pose cellB = fieldPose(CELL_B_RED);
    Pose start = robot.follower.pose();
    Pose out = fieldPose(OUT_RED).withHeading(start.heading());
    Pose shootA = shootingPose(OLD_SHOOT_A_RED, cellA);
    Pose gardenTurn = fieldPose(GARDEN_TURN_RED);
    Pose gardenWall = fieldPose(GARDEN_WALL_RED);
    Pose gardenEnd = fieldPose(GARDEN_END_RED);
    Pose shootB = shootingPose(OLD_SHOOT_B_RED, cellB);
    Pose flowerApproach = fieldPose(FLOWER_APPROACH_RED);
    Pose flower = fieldPose(FLOWER_RED);

    // --- 1. out, line up at the audience CELL, shoot all 4: the third tips the HIVE ---
    phase = "DRIVE A";
    robot.intake.setFlower(false);   // up at START, like the other autos (Intake no longer moves it at INIT)
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
    driveTo(Paths.curve(atGarden, fieldPose(OLD_VIA_1_RED), fieldPose(OLD_VIA_2_RED), shootB).linear(atGarden, shootB),
        DRIVE_TIMEOUT_MS);
    intake(false);
    phase = "AIM B";
    if (aimUntilReady(AIM_TIMEOUT_MS) || robot.limelight.hasValidTarget()) {
      phase = "SHOOT B 1";
      shootVolley();
    }

    // --- 4. the rear flower's POLLEN (flower arm down) ---
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
