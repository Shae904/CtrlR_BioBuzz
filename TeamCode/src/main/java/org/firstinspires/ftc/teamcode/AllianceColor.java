package org.firstinspires.ftc.teamcode;

public enum AllianceColor {
  RED {

    @Override
    public int getLLPipelineTeleOP() {
      return 1;
    }

    @Override
    public double getSteadyState() {
      return 270.0;   // the robot's right, like BLUE (was 90, the left: the battery side shot less consistently)
    }
  },
  BLUE {

    @Override
    public int getLLPipelineTeleOP() {
      return 2;
    }

    @Override
    public double getSteadyState() {
      return 270.0;
    }

  };

  /**
   * The alliance pipeline: both CELLS of our HIVE (tags 30-37 / 38-45). The autos run on it; the TELEOP starts on it
   * and then switches to one CELL's pipeline (Limelight.PIPELINE_*, HiveTargeting).
   */
  public abstract int getLLPipelineTeleOP();

  /**
   * The turret's rest direction and INIT position (turret position: 180 = shooting forward, 90 = left, 270 = right).
   * The turret encoder zeroes at every INIT, so the turret must be pointing this way when INIT is pressed. Both
   * alliances use the robot's right (270) since 2026-11-14; RED used the left (90) before, which put its shots on the
   * battery side.
   */
  public abstract double getSteadyState();

  /** The rest direction in words, for the INIT reminder. */
  public String turretRestName() {
    double s = getSteadyState();
    return s == 90 ? "LEFT" : s == 270 ? "RIGHT" : s == 180 ? "FORWARD" : (s + " deg");
  }
}