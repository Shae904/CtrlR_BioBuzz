package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Which of our HIVE's two CELLS the Limelight looks at in TELEOP, picked from the robot's odometry.
 *
 * <p>Field (manual 9.9, Figure 9-17; Pedro coordinates, inches, the audience wall at y = 0): each HIVE has an AUDIENCE
 * CELL whose opening and tags face the audience wall (red tags 34-37, blue 38-41) and a SCORING CELL facing the far
 * wall (red 30-33, blue 42-45). A CELL's tags can only be seen from its own side, so:
 * <ul>
 *   <li>robot in the two tile rows along the audience wall (y below {@link #AUDIENCE_ZONE_MAX_Y}): AUDIENCE CELL;</li>
 *   <li>robot in the two rows along the far wall (y above {@link #SCORING_ZONE_MIN_Y}): SCORING CELL;</li>
 *   <li>the two middle rows (the HIVES' rows): no automatic switch, the last choice holds.</li>
 * </ul>
 *
 * <p>Manual override (the two buttons): a press switches at once and pauses the automatic switching, which resumes
 * once odometry puts the robot in the outer tiles of the side the driver picked (odometry agrees again). So a wrong
 * pose cannot keep undoing the driver's choice, and pressing early (before driving over) just works.
 *
 * <p>No hardware here: {@link #update} takes the odometry y and the two button presses and returns the CELL.
 */
@Configurable
public class HiveTargeting {

  public enum Cell { AUDIENCE, SCORING }

  public enum Zone { AUDIENCE, MIDDLE, SCORING }

  /** Field interior (in): six tiles, the audience wall at y = 0. */
  public static double FIELD_IN = 141.05;
  /** Odometry y (in) below which the robot is in the two tile rows along the audience wall. */
  public static double AUDIENCE_ZONE_MAX_Y = FIELD_IN * 2 / 6;
  /** Odometry y (in) above which the robot is in the two tile rows along the far (scoring-side) wall. */
  public static double SCORING_ZONE_MIN_Y = FIELD_IN * 4 / 6;

  private Cell cell;
  private boolean auto = true;
  private Zone zone;

  /** Starts on the CELL of the zone at {@code startYIn}; in the middle rows, the CELL of the nearer half. */
  public HiveTargeting(double startYIn) {
    zone = zoneOf(startYIn);
    Cell zoneCell = cellOf(zone);
    cell = zoneCell != null ? zoneCell : (startYIn < FIELD_IN / 2 ? Cell.AUDIENCE : Cell.SCORING);
  }

  public static Zone zoneOf(double yIn) {
    if (yIn < AUDIENCE_ZONE_MAX_Y) {
      return Zone.AUDIENCE;
    }
    if (yIn > SCORING_ZONE_MIN_Y) {
      return Zone.SCORING;
    }
    return Zone.MIDDLE;
  }

  /** The CELL an outer zone targets, or null for the middle rows. */
  private static Cell cellOf(Zone zone) {
    switch (zone) {
      case AUDIENCE:
        return Cell.AUDIENCE;
      case SCORING:
        return Cell.SCORING;
      default:
        return null;
    }
  }

  /**
   * One loop.
   *
   * @param yIn             odometry y (in, Pedro field frame)
   * @param audiencePressed the "target AUDIENCE" button was pressed since the last loop
   * @param scoringPressed  the "target SCORING" button was pressed since the last loop
   * @return the CELL to target
   */
  public Cell update(double yIn, boolean audiencePressed, boolean scoringPressed) {
    zone = zoneOf(yIn);
    Cell zoneCell = cellOf(zone);
    Cell pressed = audiencePressed ? Cell.AUDIENCE : (scoringPressed ? Cell.SCORING : null);
    if (pressed != null) {
      cell = pressed;
      auto = pressed == zoneCell;        // agrees with odometry: stay automatic
    } else if (auto) {
      if (zoneCell != null) {
        cell = zoneCell;
      }
    } else if (zoneCell == cell) {
      auto = true;                       // the robot reached the tiles of the side the driver picked
    }
    return cell;
  }

  public Cell cell() {
    return cell;
  }

  /** False after a manual press, until odometry puts the robot in the outer tiles of the chosen side. */
  public boolean isAuto() {
    return auto;
  }

  /** The zone at the last {@link #update} (or the start). */
  public Zone zone() {
    return zone;
  }
}
