package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Where the turret should point, kept as a direction on the field so it survives the camera's delay, the robot
 * turning, and the tags dropping out.
 *
 * <p>Turret positions use the team's convention: 180 = shooting forward, 90 = left, 270 = right (clockwise +), so the
 * shot's field heading is {@code robotHeading + 180 - position} (deg, counter-clockwise) and a fixed field direction F
 * is the turret position {@code 180 + robotHeading - F}.
 * <ul>
 *   <li>Camera delay: a Limelight frame shows the hive as it was ~20-40 ms ago. {@link #record} keeps the turret angle
 *       and robot heading of the last loops and {@link #at} looks them up at the frame's capture time, so the new aim
 *       ({@link #visionFix}) is "turret angle then + the turn the frame asks for", not "turret angle now + an old turn"
 *       (which overshoots while the turret or the robot moves). Only new frames count.</li>
 *   <li>Robot turning: the aim is held as a field direction; {@link #targetFromField} turns it into the turret position
 *       for the robot's heading now, every loop (plus a feed-forward of the turn rate in Limelight.updateAimPID).</li>
 *   <li>Tags lost: the last field direction holds for {@link #LOST_HOLD_MS}; then the caller can point the turret at
 *       the CELL from odometry ({@link #openingBearingDeg}) until the tags are back.</li>
 *   <li>Limits: {@link #limit} picks, among the equivalent angles (+/- 360), the one inside
 *       [{@link #TURRET_MIN_DEG}, {@link #TURRET_MAX_DEG}] closest to the turret now, else the nearer limit.</li>
 * </ul>
 * No hardware here (testable on its own); Limelight drives it.
 */
@Configurable
public class TurretTracker {

  /** Turret position limits (deg, 180 = forward). The default 0..360 keeps the seam straight back (as before). */
  public static double TURRET_MIN_DEG = 0;
  public static double TURRET_MAX_DEG = 360;
  /**
   * Seam hysteresis (deg): a target up to this far past a limit holds the turret at that limit instead of swinging the
   * long way round, so a heading wobble around the seam does not flip it back and forth.
   */
  public static double SEAM_HYSTERESIS_DEG = 15;
  /** After the tags are lost, keep the last seen field direction this long (ms) before the odometry pre-aim. */
  public static int LOST_HOLD_MS = 500;
  /** A frame asking for a bigger turn than this (deg) is ignored (the tags are inside the camera's +/-27 deg view). */
  public static double MAX_FRAME_TURN_DEG = 40;
  /** Turret axis in the robot frame (in; x forward, y left; Worlds Bot CAD: 13.8 mm back, 46.25 mm right). */
  public static double AXIS_X_IN = -0.543;
  public static double AXIS_Y_IN = -1.821;

  private static final int HISTORY = 64;
  private final long[] timeNs = new long[HISTORY];
  private final double[] turretDeg = new double[HISTORY];
  private final double[] headingDeg = new double[HISTORY];
  private int size;
  private int next;

  private double fieldDeg;
  private boolean hasFix;
  private long fixNs;

  /** A new aiming session: forget the history and the last fix. */
  public void reset() {
    size = 0;
    next = 0;
    hasFix = false;
  }

  /** Every loop while aiming: the turret position and the robot heading (deg, CCW) now. */
  public void record(long nowNs, double turretPosDeg, double robotHeadingDeg) {
    timeNs[next] = nowNs;
    turretDeg[next] = turretPosDeg;
    headingDeg[next] = robotHeadingDeg;
    next = (next + 1) % HISTORY;
    size = Math.min(size + 1, HISTORY);
  }

  /** {turret position, robot heading} at {@code tNs}, interpolated; the newest / oldest sample outside the history. */
  public double[] at(long tNs) {
    if (size == 0) {
      return new double[]{Double.NaN, Double.NaN};
    }
    int newest = (next - 1 + HISTORY) % HISTORY;
    if (tNs >= timeNs[newest]) {
      return new double[]{turretDeg[newest], headingDeg[newest]};
    }
    int later = newest;
    for (int k = 1; k < size; k++) {
      int i = (newest - k + HISTORY) % HISTORY;
      if (timeNs[i] <= tNs) {
        double f = (double) (tNs - timeNs[i]) / (timeNs[later] - timeNs[i]);
        return new double[]{turretDeg[i] + f * (turretDeg[later] - turretDeg[i]),
            headingDeg[i] + f * wrapDeg(headingDeg[later] - headingDeg[i])};
      }
      later = i;
    }
    return new double[]{turretDeg[later], headingDeg[later]};   // older than the history: the oldest sample
  }

  /**
   * A new frame: at its capture the turret was at {@code turretAtCaptureDeg} with the robot at
   * {@code headingAtCaptureDeg}, and the frame asks to turn {@code turnDeg} (+ = right). False if implausible.
   */
  public boolean visionFix(long nowNs, double turretAtCaptureDeg, double headingAtCaptureDeg, double turnDeg) {
    if (Double.isNaN(turretAtCaptureDeg) || Math.abs(turnDeg) > MAX_FRAME_TURN_DEG) {
      return false;
    }
    fieldDeg = headingAtCaptureDeg + 180 - (turretAtCaptureDeg + turnDeg);
    hasFix = true;
    fixNs = nowNs;
    return true;
  }

  /** Forget the last fix (the target CELL changed). */
  public void clearFix() {
    hasFix = false;
  }

  /** A vision fix no older than LOST_HOLD_MS. */
  public boolean hasRecentFix(long nowNs) {
    return hasFix && nowNs - fixNs <= LOST_HOLD_MS * 1_000_000L;
  }

  /** The turret position that points at the last fix's field direction, for the robot's heading now. */
  public double targetFromField(double robotHeadingDeg) {
    return 180 + robotHeadingDeg - fieldDeg;
  }

  /** The turret position that shoots along field heading {@code fieldBearingDeg} (deg, CCW). */
  public static double targetFromBearing(double robotHeadingDeg, double fieldBearingDeg) {
    return 180 + robotHeadingDeg - fieldBearingDeg;
  }

  /** Field heading (deg, CCW) from the turret axis to the point (in), for a robot at (x, y) in with this heading. */
  public static double openingBearingDeg(double robotXIn, double robotYIn, double robotHeadingRad, double pointXIn,
                                         double pointYIn) {
    double c = Math.cos(robotHeadingRad);
    double s = Math.sin(robotHeadingRad);
    double axisX = robotXIn + AXIS_X_IN * c - AXIS_Y_IN * s;
    double axisY = robotYIn + AXIS_X_IN * s + AXIS_Y_IN * c;
    return Math.toDegrees(Math.atan2(pointYIn - axisY, pointXIn - axisX));
  }

  /**
   * The equivalent of {@code target} (+/- 360) closest to {@code turretNow} among those inside the limits widened by
   * SEAM_HYSTERESIS_DEG, clamped to the limits (so just past the seam the turret waits at the limit on its side);
   * else the nearer limit.
   */
  public static double limit(double target, double turretNow) {
    double best = Double.NaN;
    double base = target - 360 * Math.floor((target - TURRET_MIN_DEG) / 360);   // in [MIN, MIN + 360)
    double lo = TURRET_MIN_DEG - SEAM_HYSTERESIS_DEG;
    double hi = TURRET_MAX_DEG + SEAM_HYSTERESIS_DEG;
    for (int k = -1; k <= 1; k++) {
      double c = base + 360 * k;
      if (c >= lo - 1e-9 && c <= hi + 1e-9
          && (Double.isNaN(best) || Math.abs(c - turretNow) < Math.abs(best - turretNow))) {
        best = c;
      }
    }
    if (!Double.isNaN(best)) {
      return Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, best));
    }
    // Out of reach: the limit the target is angularly nearer to
    double toMax = Math.abs(wrapDeg(base - TURRET_MAX_DEG));
    double toMin = Math.abs(wrapDeg(base - TURRET_MIN_DEG));
    return toMax <= toMin ? TURRET_MAX_DEG : TURRET_MIN_DEG;
  }

  static double wrapDeg(double d) {
    return d - 360 * Math.floor((d + 180) / 360);
  }
}
