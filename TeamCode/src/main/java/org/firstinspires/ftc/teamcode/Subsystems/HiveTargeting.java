package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Which of our HIVE's two CELLS the Limelight looks at in TELEOP: chosen by the drivers (gamepad2 circle / Xbox B
 * toggles AUDIENCE / SCORING, BaseTeleop). There is no automatic switching; at INIT the teleop starts on the CELL of the field half
 * the robot is in ({@link #initialCell}), which after an AUTO is the side it parked on.
 *
 * <p>Field (manual 9.9, Figure 9-17; Pedro coordinates, inches, the audience wall at y = 0): each HIVE has an AUDIENCE
 * CELL whose opening and tags face the audience wall (red tags 34-37, blue 38-41) and a SCORING CELL facing the far
 * wall (red 30-33, blue 42-45). A CELL's tags can only be seen from its own side.
 *
 * <p>Odometry reset at the audience wall ({@link #audienceWallY}): with the robot pushed against the audience wall
 * anywhere along it, y is known from the robot's footprint and heading (x and heading are kept). The odometry is only
 * used for the turret's pre-aim when no tags are in view.
 *
 * <p>No hardware here.
 */
@Configurable
public class HiveTargeting {

  public enum Cell { AUDIENCE, SCORING }

  /** Field interior (in): six tiles, the audience wall at y = 0. */
  public static double FIELD_IN = 141.05;

  // Robot footprint from its centre (in; Worlds Bot CAD, biobuzz-sim docs/research/07): what touches the wall
  /** Front (the intake wheels), 250.8 mm. */
  public static double FRONT_IN = 9.874;
  /** Back, 186.9 mm (backed against the audience wall at 90 deg: y 7.36, the autos' start). */
  public static double BACK_IN = 7.358;
  /** Left, 212.65 mm. */
  public static double LEFT_IN = 8.372;
  /** Right (the turret's input assembly), 242.15 mm. */
  public static double RIGHT_IN = 9.533;

  private HiveTargeting() {
  }

  /** The CELL facing the field half the robot is in: AUDIENCE below mid-field, SCORING above. */
  public static Cell initialCell(double yIn) {
    return yIn < FIELD_IN / 2 ? Cell.AUDIENCE : Cell.SCORING;
  }

  /**
   * The robot centre's y (in) with the robot pushed against the audience wall (y = 0) at this heading (rad, Pedro):
   * the footprint's lowest point touches the wall, whichever face or corner that is.
   */
  public static double audienceWallY(double headingRad) {
    double s = Math.sin(headingRad);
    double c = Math.cos(headingRad);
    double lowest = Double.POSITIVE_INFINITY;
    for (double xr : new double[]{FRONT_IN, -BACK_IN}) {
      for (double yr : new double[]{LEFT_IN, -RIGHT_IN}) {
        lowest = Math.min(lowest, xr * s + yr * c);   // field y of a footprint corner, relative to the centre
      }
    }
    return -lowest;
  }
}
