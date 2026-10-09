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
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Intake {

  // Color Sensor tuning vars


  public static final double POWER_INTAKE = 1;

  public static Direction intakeMotorDirection = Direction.REVERSE;
  public final DcMotor intakeMotor;


  private enum CycleState {PENDING, OPEN, CLOSE, CLOSE_DELAY}

  public static int OPEN_DELAY = 400;

  public static int CLOSE_DELAY = 500;

  public static double FLAPPER_SHOOT = 0.1;

  public static double FLAPPER_LOCKED = 0.35; //.45

  public static double CYCLER_LOCKED = .63;


  private CycleState cycleState = CycleState.PENDING;

  ElapsedTime cycleTimer = new ElapsedTime();

  private int remainingCycles = 0;


  public ServoImplEx gate;
  public ServoImplEx cycler;

  public enum FlapperState {CYCLE, SHOOT, LOCKED}

  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

    intakeMotor.setDirection(intakeMotorDirection);

    intakeMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    intakeMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);

    gate = hardwareMap.get(ServoImplEx.class, "flapper");
    cycler = hardwareMap.get(ServoImplEx.class, "cycler");
  }

  public void setCyclePosition(FlapperState state) {
    switch (state) {
      case CYCLE:

        break;
      case SHOOT:
        this.setFlapperPos(FLAPPER_SHOOT);
        this.setCyclerPos(CYCLER_LOCKED);
        break;
      case LOCKED:
        this.setFlapperPos(FLAPPER_LOCKED);
        this.setCyclerPos(CYCLER_LOCKED);
        break;
    }
  }

  public void setFlapperPos(double pos) {
    gate.setPosition(pos);
  }

  public void setCyclerPos(double pos) {
    cycler.setPosition(pos);
  }

  public void setIntakePower(double pow) {
    intakeMotor.setPower(pow);
  }

  public void cycle(int num) {
    remainingCycles = num;
  }

  public void cycleIncrementByNum(int num) {
    remainingCycles += num;
  }

  public boolean isCycleFinished() {
    return this.cycleState == CycleState.PENDING;
  }


  public void updateAutoCycle() {
    switch (cycleState) {
      case PENDING:
        if (remainingCycles > 0) {
          this.cycleState = CycleState.OPEN;
        }
        break;
      case OPEN:
        remainingCycles -= 1;
        this.setCyclePosition(FlapperState.CYCLE);
        this.cycleTimer.reset();
        this.cycleState = CycleState.CLOSE;
        break;
      case CLOSE:
        if (this.cycleTimer.milliseconds() >= OPEN_DELAY) {
          setCyclePosition(FlapperState.LOCKED);
          this.cycleTimer.reset();
          this.cycleState = CycleState.CLOSE_DELAY;
        }
        break;
      case CLOSE_DELAY:
        if (this.cycleTimer.milliseconds() >= CLOSE_DELAY) {
          if (remainingCycles == 0) {
            this.cycleState = CycleState.PENDING;
          } else {
            this.cycleState = CycleState.OPEN;
          }

        }

        break;
    }
  }

  public void cancelCycle() {
    this.remainingCycles = 0;
    this.cycleState = CycleState.PENDING;
    setCyclePosition(FlapperState.LOCKED);
  }

}
