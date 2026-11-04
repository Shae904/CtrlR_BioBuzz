package org.firstinspires.ftc.teamcode;

public enum AllianceColor {
  RED {

    @Override
    public int getLLPipelineTeleOP() {
      return 1;
    }

    @Override
    public double getSteadyState() {
      return 90.0;
    }


    ;
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

  public abstract double getSteadyState();
}