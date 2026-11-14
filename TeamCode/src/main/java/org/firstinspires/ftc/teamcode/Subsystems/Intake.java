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
   * autos); DOWN = shut, the hopper holds its balls while intaking. It starts DOWN at INIT. Live-tunable in Panels.
   * Measured on the slapper 2026-10-22 (CommanderSleepy): 0.27 up, 0.1 down.
   */
  public static double FLAPPER_UP = 0.62;

  public static double FLAPPER_DOWN = 0.78;

  public static double FLOWER_UP = 0.3;
  public static double FLOWER_DOWN = 0.74;
  public static double FLOWER_TWO_UP = 0.29;
  public static double FLOWER_TWO_DOWN = 0.58;


  public final ServoImplEx gate,flower,flower2;
  private boolean flapperUp;
  private boolean flowerDown;

  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    intakeMotor.setDirection(intakeMotorDirection);
    intakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    intakeMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);

    gate = hardwareMap.get(ServoImplEx.class, "flapper");
    flower = hardwareMap.get(ServoImplEx.class, "flower");
    flower2 = hardwareMap.get(ServoImplEx.class, "flower2");
    setFlapper(false); // starts DOWN (shut)
    setFlowerStart();  // starts UP
  }

  /** Raises (true, open: feed) or lowers (false, shut: hold) the flapper. */
  public void setFlapper(boolean up) {
    flapperUp = up;
    gate.setPosition(up ? FLAPPER_UP : FLAPPER_DOWN);
  }
  public void setFlower(boolean down) {
    flowerDown = down;
    setFlowerPos(down ? FLOWER_DOWN : FLOWER_UP,down ? FLOWER_TWO_DOWN : FLOWER_TWO_UP);
  }
  public void setFlowerStart(){
    setFlowerPos(FLOWER_UP,FLOWER_TWO_UP);
  }

  public boolean isFlapperUp() {
    return flapperUp;
  }

  /** Raw servo position (MOTOR TESTING); {@link #setFlapper(boolean)} is the normal way. */
  public void setFlapperPos(double pos) {
    gate.setPosition(pos);
  }
  public void setFlowerPos(double pos,double pos2) {
    flower.setPosition(pos);
    flower2.setPosition(pos2);
  }

  public void setIntakePower(double pow) {
    intakeMotor.setPower(pow);
  }
}