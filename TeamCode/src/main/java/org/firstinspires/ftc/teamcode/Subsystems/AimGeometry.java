package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Turret aim from the measured tag positions: the turn that puts the shot line through the CELL opening.
 *
 * <p>Why not a fixed tx offset: the Limelight sits {@link #CAM_RIGHT_OF_SHOT_M} right of the shot line and
 * {@link #CAM_BEHIND_AXIS_M} behind the turret axis, and the opening is {@link #OPENING_TOWARD_SHOOTER_M} in front of
 * the tag cluster along the CELL's axis, so the tx at which the shot goes in changes with distance and angle (about
 * -10 to -0.4 deg across one side of the field). And averaging whatever tags are visible moves the aim when a tag drops
 * out (one outer tag: ~2.3 deg; one pair only: ~5 deg).
 *
 * <p>So: every visible tag gives its position (horizontal bearing + range, level camera frame) and, from its known
 * place in the row ({@link #TAG_ROW_OFFSET_M}), an estimate of the cluster centre; their mean is the centre whichever
 * tags are seen. The opening is the centre moved toward the shooter; in the turret frame that gives the exact turn
 * and the distance for the shot formula.
 *
 * <p>Field facts (field CAD, biobuzz-sim tag model): a cluster's ids firstID + 0..3 run left to right as seen from
 * the side the CELL faces, at -6.5, -2.75, +2.75, +6.5 in along the hive's pivot axis (field x). Seen from the front,
 * "right" is field +x for the AUDIENCE CELLS (they face the audience wall, -y) and -x for the SCORING CELLS (+y). The
 * row direction and the opening's step from the cluster, in the camera frame, come from the camera's field heading
 * (robot heading + turret angle): a 5 deg error there moves the aim ~6 mm with all 4 tags, ~9 mm with 3, ~14 mm with
 * one pair (the opening leaves +/-73..220 mm), so the turret must start at its rest direction at INIT (the encoder
 * zeroes there).
 *
 * <p>No hardware here (testable on its own); {@code Limelight.updateHiveTarget} feeds it.
 */
@Configurable
public class AimGeometry {

  // Turret frame (ghettoturret v10 CAD): x along the shot line, y to the left, origin on the turret axis
  /** Limelight lens behind the turret axis, along the shot line (m). */
  public static double CAM_BEHIND_AXIS_M = 0.0644;
  /** Limelight lens right of the shot line (m). */
  public static double CAM_RIGHT_OF_SHOT_M = 0.0815;
  /** The ball leaves the hood this far ahead of the turret axis, on the shot line (m). */
  public static double LAUNCH_AHEAD_OF_AXIS_M = 0.0048;
  /** CELL opening centre vs its tag cluster centre: this far toward the shooter, along the CELL's axis (m). */
  public static double OPENING_TOWARD_SHOOTER_M = 0.073;
  /** Half the CELL opening's clear width (m; field CAD: 512 mm between the side skins). */
  public static double OPENING_HALF_WIDTH_M = 0.256;
  /** POLLEN radius (m; 2.80 in). */
  public static double BALL_RADIUS_M = 0.0356;

  /** Tag centres along the cluster's row, ids firstID + 0..3, + = right as seen from the front (m). */
  static final double[] TAG_ROW_OFFSET_M = {-6.5 * 0.0254, -2.75 * 0.0254, 2.75 * 0.0254, 6.5 * 0.0254};

  /** One CELL's visible tags this frame. */
  public static final class Cluster {
    private double sumF;
    private double sumR;
    private double sumRow;
    private int count;

    /**
     * @param indexInCell the tag's place in its cluster, (id - 30) % 4
     * @param bearingRad  horizontal bearing from the lens, level frame, + = right
     * @param rangeM      horizontal distance from the lens
     */
    public void add(int indexInCell, double bearingRad, double rangeM) {
      sumF += rangeM * Math.cos(bearingRad);
      sumR += rangeM * Math.sin(bearingRad);
      sumRow += TAG_ROW_OFFSET_M[indexInCell];
      count++;
    }

    public int count() {
      return count;
    }

    public void clear() {
      sumF = 0;
      sumR = 0;
      sumRow = 0;
      count = 0;
    }
  }

  /** The answer for one frame. */
  public static final class Aim {
    /** Turret turn that puts the shot line through the opening (deg, + = right, the turret position's sense). */
    public final double correctionDeg;
    /** The shot formula's distance: camera to opening along the shot line once aimed (= launch to opening + 69 mm). */
    public final double formulaDistanceM;
    /** Lens to the tag cluster's centre, horizontal (m), for telemetry. */
    public final double tagDistanceM;
    /**
     * How far (deg, either side) the shot line may be off and the ball still pass the opening's sides: the opening's
     * width as seen from here (narrower off its axis) minus the ball, over the distance.
     */
    public final double windowDeg;

    Aim(double correctionDeg, double formulaDistanceM, double tagDistanceM, double windowDeg) {
      this.correctionDeg = correctionDeg;
      this.formulaDistanceM = formulaDistanceM;
      this.tagDistanceM = tagDistanceM;
      this.windowDeg = windowDeg;
    }
  }

  private AimGeometry() {
  }

  /**
   * @param cluster      the CELL's visible tags (at least one)
   * @param audienceCell the CELL faces the audience wall (ids 34-37, 38-41); false for the far wall (30-33, 42-45)
   * @param cameraYawRad the camera's (= the shot line's) heading on the field, rad CCW: robot heading + turret angle
   */
  public static Aim solve(Cluster cluster, boolean audienceCell, double cameraYawRad) {
    double n = cluster.count;
    double s = audienceCell ? 1 : -1;
    double cos = Math.cos(cameraYawRad);
    double sin = Math.sin(cameraYawRad);
    // Row direction (right as seen from the front: field (s, 0)) and the opening's direction from the cluster
    // (toward the shooter: field (0, -s)), both in the level camera frame (forward, right)
    double rowF = s * cos;
    double rowR = s * sin;
    double outF = -s * sin;
    double outR = s * cos;
    // Cluster centre: each tag's position minus its offset along the row, averaged
    double meanRow = cluster.sumRow / n;
    double centreF = cluster.sumF / n - meanRow * rowF;
    double centreR = cluster.sumR / n - meanRow * rowR;
    double openF = centreF + OPENING_TOWARD_SHOOTER_M * outF;
    double openR = centreR + OPENING_TOWARD_SHOOTER_M * outR;
    // Opening in the turret frame (x along the shot line, y left) and the turn that brings the shot line onto it
    double x = openF - CAM_BEHIND_AXIS_M;
    double y = -(openR + CAM_RIGHT_OF_SHOT_M);
    double correctionDeg = -Math.toDegrees(Math.atan2(y, x));
    // Once turned, the launch point is LAUNCH_AHEAD_OF_AXIS_M along the line to the opening, and the formula was fitted
    // on the camera's distance with the camera CAM_BEHIND_AXIS_M + LAUNCH_AHEAD_OF_AXIS_M (69 mm) behind the launch
    double launchToOpening = Math.hypot(x, y) - LAUNCH_AHEAD_OF_AXIS_M;
    double formulaDistance = launchToOpening + LAUNCH_AHEAD_OF_AXIS_M + CAM_BEHIND_AXIS_M;
    // Window: the angle between the line of sight (opening -> turret) and the CELL's axis narrows the opening
    double cosOff = Math.abs(-x * outF + y * outR) / Math.hypot(x, y);   // the axis in the turret frame: (outF, -outR)
    double half = OPENING_HALF_WIDTH_M * cosOff - BALL_RADIUS_M;
    double windowDeg = Math.toDegrees(Math.atan2(Math.max(0, half), launchToOpening));
    return new Aim(correctionDeg, formulaDistance, Math.hypot(centreF, centreR), windowDeg);
  }
}
