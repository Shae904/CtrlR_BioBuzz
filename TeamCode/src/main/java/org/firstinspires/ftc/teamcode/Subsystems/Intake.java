package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/** The intake roller and the flapper (the "slapper" gate between the hopper and the flywheel). */
@Configurable
public class Intake {

  public static final double POWER_INTAKE = 1;

  public static Direction intakeMotorDirection = Direction.REVERSE;
  public final DcMotor intakeMotor;

  /**
   * The flapper has two positions: UP = open, balls feed into the flywheel (held triangle in teleop, shooting in the
   * autos); DOWN = shut, the hopper holds its balls while intaking. It starts DOWN at INIT. Live-tunable in Panels: if
   * the slapper moves the wrong way on the new mount, swap these two numbers (DECODE's values until measured on this
   * robot).
   */
  public static double FLAPPER_UP = 0.27;

  public static double FLAPPER_DOWN = 0.1;

  public final ServoImplEx gate;
  private boolean flapperUp;

  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    intakeMotor.setDirection(intakeMotorDirection);
    intakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    intakeMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);

    gate = hardwareMap.get(ServoImplEx.class, "flapper");
    setFlapper(false);   // starts DOWN (shut)
  }

  /** Raises (true, open: feed) or lowers (false, shut: hold) the flapper. */
  public void setFlapper(boolean up) {
    flapperUp = up;
    gate.setPosition(up ? FLAPPER_UP : FLAPPER_DOWN);
  }

  public boolean isFlapperUp() {
    return flapperUp;
  }

  /** Raw servo position (MOTOR TESTING); {@link #setFlapper(boolean)} is the normal way. */
  public void setFlapperPos(double pos) {
    gate.setPosition(pos);
  }

  public void setIntakePower(double pow) {
    intakeMotor.setPower(pow);
  }
}
