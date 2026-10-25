package org.firstinspires.ftc.teamcode;

public enum AllianceColor {
  RED {

    @Override
    public int getLLPipelineTeleOP() {
      return 1;
    }

    public int getLLPipelineAuto() { return 3; }

    @Override
    public int getAimPose() {
      return -5;
    } //-3

    @Override
    public double getSteadyState() {
      return 90.0;
    }


    ;
  },
  BLUE {

    @Override
    public int getLLPipelineTeleOP() {
      return 2; // TODO: make these in LL
    }

    public int getLLPipelineAuto() { return 4; }

    @Override
    public int getAimPose() {
      return -1;
    } //-2

    @Override
    public double getSteadyState() {
      return 270.0;
    }

  };

  public abstract int getLLPipelineTeleOP();

  public abstract  int getLLPipelineAuto();

  public abstract int getAimPose();

  public abstract double getSteadyState();
}