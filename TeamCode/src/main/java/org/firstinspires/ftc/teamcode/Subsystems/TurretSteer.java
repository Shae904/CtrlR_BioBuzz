package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The TELEOP's turret target (the autos keep their own, Limelight.updateTarget, with its rest direction).
 *
 * <p>The turret no longer returns to its rest direction in the TELEOP: when nobody aims it stays where it last did
 * something useful (PARKED). Turret positions as everywhere: 180 = shooting forward, 90 = left, 270 = right.
 * <ul>
 *   <li><b>Semi-manual, gamepad2's right stick.</b> Off its centre in any direction = aiming. Without tags the turret
 *       follows the stick's left/right (MANUAL). As soon as the target CELL's tags are seen the Limelight takes over
 *       (VISION, or HOLD while they drop out) and the stick's position is ignored; the lock lasts as long as the stick
 *       stays off its centre. Centring the stick stops the aiming and the turret stays put.</li>
 *   <li><b>Lock lost</b> (no tags for {@link #LOCK_LOST_HOLD_MS}): the turret freezes (LOST) and the stick does not
 *       steer again until it has been centred once; else a stick held over to keep the lock would run the turret away
 *       the moment the lock drops. Tags back while the stick is still held: locked again.</li>
 *   <li><b>RB</b> (either pad, without the stick): as before (VISION / HOLD, then the odometry pre-aim), but when the
 *       pre-aim finds nothing the turret goes back to where it was parked, not to the rest direction.</li>
 *   <li><b>The cable.</b> The stick never takes the turret past its limits (TurretTracker.TURRET_MIN/MAX_DEG, the seam
 *       straight back) nor swings it round: it stops {@link #STEER_LIMIT_MARGIN_DEG} short, steer back the other way.
 *       A lock keeps TurretTracker.limit: wait at the limit up to the hysteresis, then the long way round.</li>
 * </ul>
 * No hardware here (testable on its own); Limelight.updateTargetTeleop drives it.
 */
@Configurable
public class TurretSteer {

  /** Full stick turns the turret this fast (deg/s); the stick is squared, so half stick is a quarter of it. */
  public static double STEER_MAX_DEG_S = 200;
  /** Swaps the stick's direction (default: stick right = the turret turns to its right, the position going up). */
  public static boolean STEER_INVERT = false;
  /** The stick stops the turret this far (deg) inside its limits: room for the overshoot, the cable never past them. */
  public static double STEER_LIMIT_MARGIN_DEG = 5;
  /**
   * Toward a limit the stick's speed is capped at this (deg/s) per degree of room left (6: full speed until 33 deg
   * before the stop, then slowing), and never under STEER_MIN_DEG_S, so the turret does not overshoot into the cable.
   */
  public static double STEER_SLOW_PER_DEG = 6;
  public static double STEER_MIN_DEG_S = 20;
  /** The steering target never runs further ahead of the turret than this (deg): no wind-up against a stalled turret. */
  public static double STEER_MAX_LEAD_DEG = 25;
  /**
   * Locked from the stick and the tags are gone: the last field direction is kept this long (ms) before the lock is
   * given up (LOST). Longer than the 500 ms of RB (TurretTracker.LOST_HOLD_MS) because there is no odometry pre-aim
   * behind it: it has to cover a HIVE tip and the swing the long way round at the cable seam.
   */
  public static int LOCK_LOST_HOLD_MS = 1500;
  /** When aiming stops the turret parks at its target if that is within this (deg), else this far ahead of itself. */
  public static double PARK_AHEAD_DEG = 10;

  public static final String PARKED = "PARKED";
  public static final String MANUAL = "MANUAL";
  public static final String VISION = "VISION";
  public static final String HOLD = "HOLD";
  public static final String LOST = "LOST";
  public static final String ODOMETRY = "ODOMETRY";
  /** RB held, no tags, the odometry pre-aim gave up: back at the parked direction (re-press RB to retry). */
  public static final String PARK_SEARCH = "PARK-SEARCH";

  private double park = Double.NaN;       // where the turret stays when nobody aims
  private double steerTarget;
  private boolean steering;               // MANUAL last loop (the steering target is live)
  private boolean stickLocked;            // locked from the stick last loop (the longer hold applies)
  private boolean armed = true;           // the stick was centred since the last lock: it may steer
  private boolean rbWasOnly;
  private long preaimSinceNs = -1;
  private long lastNs;
  private String source = PARKED;
  private double steerRateDegS;
  private boolean atLimit;

  /**
   * One TELEOP loop.
   *
   * @param nowNs           the time (ns)
   * @param turretNow       the turret position (deg)
   * @param rbAim           RB held (either pad): the automatic aim
   * @param stickX          gamepad2's right stick, left/right (-1..1, + = right)
   * @param stickActive     that stick is off its centre, in any direction
   * @param fixAgeMs        ms since the last vision fix (TurretTracker.fixAgeMs; infinity = none)
   * @param tagsNow         the target CELL's tags are in the current frame
   * @param fieldTarget     the turret position of the last fix's field direction for the robot's heading now
   * @param odometryTarget  the turret position toward the target CELL from odometry, NaN = none
   * @param preaimTimeoutMs how long the odometry pre-aim may run per RB press without tags
   * @return the turret target (deg), inside the turret's limits
   */
  public double update(long nowNs, double turretNow, boolean rbAim, double stickX, boolean stickActive,
                       double fixAgeMs, boolean tagsNow, double fieldTarget, double odometryTarget,
                       int preaimTimeoutMs) {
    double dt = lastNs == 0 ? 0 : Math.min((nowNs - lastNs) / 1e9, 0.1);
    lastNs = nowNs;
    double here = clampToLimits(turretNow, 0);
    if (Double.isNaN(park)) {
      park = here;                        // the TELEOP starts with the turret staying where it is
    }
    steerRateDegS = 0;
    atLimit = false;
    boolean fresh = fixAgeMs <= TurretTracker.LOST_HOLD_MS;
    boolean rbOnly = rbAim && !stickActive;
    if (rbOnly && !rbWasOnly) {
      preaimSinceNs = -1;                 // a new press: the odometry pre-aim may try again
    }
    rbWasOnly = rbOnly;
    if (!stickActive) {
      armed = true;
      stickLocked = false;
    }

    double target;
    if (stickActive && (stickLocked ? fixAgeMs <= LOCK_LOST_HOLD_MS : fresh)) {
      // the Limelight has it: the stick's position no longer matters, only that it is held
      stickLocked = true;
      steering = false;
      armed = false;
      target = TurretTracker.limit(fieldTarget, turretNow);
      park = parkNear(target, here);
      source = tagsNow ? VISION : HOLD;
    } else if (stickActive && armed) {
      stickLocked = false;
      if (!steering) {
        steering = true;
        steerTarget = here;
      }
      double x = Math.max(-1, Math.min(1, stickX));
      double rate = x * Math.abs(x) * STEER_MAX_DEG_S * (STEER_INVERT ? -1 : 1);
      // slower into a limit: the speed allowed falls with the room left, so the turret arrives without overshoot
      double room = rate > 0 ? TurretTracker.TURRET_MAX_DEG - STEER_LIMIT_MARGIN_DEG - steerTarget
          : steerTarget - TurretTracker.TURRET_MIN_DEG - STEER_LIMIT_MARGIN_DEG;
      double allowed = Math.max(STEER_MIN_DEG_S, Math.max(0, room) * STEER_SLOW_PER_DEG);
      rate = Math.max(-allowed, Math.min(allowed, rate));
      double before = steerTarget;
      double asked = steerTarget + rate * dt;
      steerTarget = Math.max(turretNow - STEER_MAX_LEAD_DEG, Math.min(turretNow + STEER_MAX_LEAD_DEG, asked));
      steerTarget = clampToLimits(steerTarget, STEER_LIMIT_MARGIN_DEG);
      atLimit = (rate > 0 && asked > TurretTracker.TURRET_MAX_DEG - STEER_LIMIT_MARGIN_DEG)
          || (rate < 0 && asked < TurretTracker.TURRET_MIN_DEG + STEER_LIMIT_MARGIN_DEG);
      if (dt > 0) {
        steerRateDegS = Math.max(-Math.abs(rate), Math.min(Math.abs(rate), (steerTarget - before) / dt));
      }
      target = steerTarget;
      park = parkNear(target, here);
      source = MANUAL;
    } else if (stickActive) {
      // the lock dropped with the stick still held: frozen at the last locked direction until the stick is centred
      stickLocked = false;
      steering = false;
      target = park;
      source = LOST;
    } else if (rbAim && fresh) {
      steering = false;
      preaimSinceNs = -1;                 // tags seen: a later pre-aim gets its full time again
      target = TurretTracker.limit(fieldTarget, turretNow);
      park = parkNear(target, here);
      source = tagsNow ? VISION : HOLD;
    } else if (rbAim && !Double.isNaN(odometryTarget) && preaimAllowed(nowNs, preaimTimeoutMs)) {
      steering = false;
      target = TurretTracker.limit(odometryTarget, turretNow);
      source = ODOMETRY;
    } else {
      steering = false;
      target = park;
      source = rbAim ? PARK_SEARCH : PARKED;
    }
    return target;
  }

  /** Odometry pre-aim allowed: within the timeout of its start in this RB press (a fix restarts it). */
  private boolean preaimAllowed(long nowNs, int timeoutMs) {
    if (preaimSinceNs < 0) {
      preaimSinceNs = nowNs;
    }
    return nowNs - preaimSinceNs <= timeoutMs * 1_000_000L;
  }

  /** Where the turret would stay if aiming stopped now: the target when close, else a short way ahead of itself. */
  private static double parkNear(double target, double here) {
    double ahead = Math.max(-PARK_AHEAD_DEG, Math.min(PARK_AHEAD_DEG, target - here));
    return clampToLimits(here + ahead, 0);
  }

  /** {@code deg} inside the turret's limits, {@code margin} short of each. */
  static double clampToLimits(double deg, double margin) {
    double lo = TurretTracker.TURRET_MIN_DEG + margin;
    double hi = TurretTracker.TURRET_MAX_DEG - margin;
    if (lo > hi) {
      lo = hi = (lo + hi) / 2;
    }
    return Math.max(lo, Math.min(hi, deg));
  }

  /** Where the target comes from: PARKED, MANUAL, VISION, HOLD, LOST, ODOMETRY or PARK-SEARCH. */
  public String source() {
    return source;
  }

  /** The Limelight is aiming the turret (tags in view, or just lost and their direction held). */
  public boolean isLocked() {
    return VISION.equals(source) || HOLD.equals(source);
  }

  /** How fast the stick is moving the target right now (deg/s, 0 outside MANUAL or against a limit). */
  public double steerRateDegS() {
    return steerRateDegS;
  }

  /** The stick is pushing the turret against its limit (the cable seam): steer the other way. */
  public boolean atLimit() {
    return atLimit;
  }

  /** Where the turret stays when nobody aims (deg). */
  public double parkDeg() {
    return park;
  }
}
