package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Carries the robot's field pose from AUTO into TELEOP (static fields live as long as the Robot Controller app).
 *
 * <p>The autos save their Pedro pose every loop ({@code BaseHiveAuto.update}); the TELEOP takes it at INIT when the
 * AUTO ran at most {@link #MAX_AGE_S} ago, as in a match, and otherwise assumes the robot sits at the alliance's AUTO
 * start spot ({@link #defaultStart}). The TELEOP takes it once, so a second TELEOP later does not reuse an old AUTO
 * pose. Needed because Pedro starts every opmode at (0, 0) and the TELEOP's Limelight targeting
 * ({@code Subsystems.HiveTargeting}) works from the field position.
 */
@Configurable
public class PoseStorage {

  /** An AUTO pose is used by the next TELEOP when it was saved at most this long ago (s). */
  public static double MAX_AGE_S = 180;
  /** The TELEOP's start pose when no AUTO ran just before: the autos' RED start (x in, y in, heading deg). */
  public static double[] DEFAULT_START_RED = {46.0, 7.36, 90};
  /** Field interior (in): BLUE's start is RED's rotated 180 degrees about the centre, like the autos' poses. */
  public static double FIELD_IN = 141.05;

  private static volatile Pose autoPose;
  private static volatile long savedAtNs;

  private PoseStorage() {
  }

  /** AUTO: the robot's current pose (call every loop). */
  public static void saveAutoPose(Pose pose) {
    autoPose = pose;
    savedAtNs = System.nanoTime();
  }

  /** Seconds since the AUTO pose was saved, or infinity when there is none. */
  public static double autoPoseAgeS() {
    return autoPose == null ? Double.POSITIVE_INFINITY : (System.nanoTime() - savedAtNs) / 1e9;
  }

  /** TELEOP INIT: the AUTO's last pose if it is recent enough, else null; either way it is used up. */
  public static Pose takeAutoPose() {
    Pose pose = autoPoseAgeS() <= MAX_AGE_S ? autoPose : null;
    autoPose = null;
    return pose;
  }

  /** The alliance's AUTO start spot. */
  public static Pose defaultStart(AllianceColor color) {
    double h = Math.toRadians(DEFAULT_START_RED[2]);
    if (color == AllianceColor.BLUE) {
      return new Pose(FIELD_IN - DEFAULT_START_RED[0], FIELD_IN - DEFAULT_START_RED[1], h + Math.PI);
    }
    return new Pose(DEFAULT_START_RED[0], DEFAULT_START_RED[1], h);
  }
}
