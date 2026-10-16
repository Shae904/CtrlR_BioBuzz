package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

  // --- PID constants (tune in Panels) ---

  // BIOBUZZ presets from the shot model (Limelight.SHOT_*): med ~2.0 m, far ~3.2 m camera -> CELL opening
  // (DECODE values were 1280 / 1640)
  public static int medSpeed = 1380;
  public static int farSpeed = 1560;

  public static int cycleSpeed = 300;

  // power = (kS + kV * target + kP * error) * NOMINAL_VOLTAGE / battery voltage
  // kS and kV come from the feedforward sweep in FLYWHEEL PID TESTING (triangle); then tune kP with its step test.
  public static double kS = 0.0;          // power to overcome friction (12 V units)
  public static double kV = 0.0004348;    // power per tick/s (12 V units)
  public static double kP = 0.003;        // power per tick/s of error
  public static boolean VOLTAGE_COMPENSATION = true; // same flywheel speed on a full or a tired battery
  public static double NOMINAL_VOLTAGE = 12.0;


  public static Direction flywheel1MotorDirection = Direction.FORWARD;
  public static Direction flywheel2MotorDirection = Direction.REVERSE;


  // --- Variables ---
  private double targetVelocity = 0; // ticks/sec

  // --- Hardware ---
  public DcMotorEx flywheel1;
  public DcMotorEx flywheel2;

  public DcMotorEx turret;


  private double currentVelocity;
  private double lastPower;

  private final VoltageSensor battery;
  private final ElapsedTime voltageTimer = new ElapsedTime();
  private double batteryVoltage = 12.0;

  // --- Constructor ---
  public Outtake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
    flywheel2.setDirection(flywheel2MotorDirection);
    flywheel2.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel2.setMode(RunMode.RUN_WITHOUT_ENCODER);

    flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
    flywheel1.setDirection(flywheel1MotorDirection);
    flywheel1.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel1.setMode(RunMode.RUN_WITHOUT_ENCODER);

    turret = hardwareMap.get(DcMotorEx.class, "turret");
    turret.setDirection(Direction.REVERSE);
    turret.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    turret.setMode(RunMode.STOP_AND_RESET_ENCODER);
    turret.setMode(RunMode.RUN_WITHOUT_ENCODER);

    battery = hardwareMap.voltageSensor.iterator().next();
    readBatteryVoltage();
  }

  private void readBatteryVoltage() {
    double v = battery.getVoltage();
    if (v > 7.0) { // ignore a bad read (0 V) instead of dividing by it
      batteryVoltage = Range.clip(v, 9.0, 15.0);
    }
    voltageTimer.reset();
  }

  /** Battery voltage, re-read at most every 250 ms (a voltage read is a separate, slow hub command). */
  public double getBatteryVoltage() {
    if (voltageTimer.milliseconds() > 250) {
      readBatteryVoltage();
    }
    return batteryVoltage;
  }


  public void setPower(double pow) {
    this.flywheel1.setPower(pow);
    this.flywheel2.setPower(pow);
  }

  // --- Set target velocity ---
  public void setTargetVelocity(double targetTicksPerSec) {
    targetVelocity = Math.max(targetTicksPerSec, 0);
  }


  // --- Main PID update loop ---
  public double updatePIDControl() {
    this.currentVelocity = this.flywheel1.getVelocity(); // ticks/sec
    double error = this.targetVelocity - this.currentVelocity;

    double output = (kV * this.targetVelocity) + (kP * error);
    if (this.targetVelocity > 0) {
      output += kS;
    }
    if (VOLTAGE_COMPENSATION) {
      output *= NOMINAL_VOLTAGE / getBatteryVoltage();
    }

    // limit power range (a little reverse power brakes the flywheel when the target drops)
    output = Range.clip(output, -.2, 1.0);

    this.setPower(output);
    this.lastPower = output;
    return output;
  }

  public double getLastPower() {
    return this.lastPower;
  }

  public double getCurrentVelocity() {
    return this.currentVelocity;
  }

  public boolean atTarget() {
    return atTarget(40);
  }

  public boolean atTarget(double threshold) {
    return Math.abs(this.getCurrentVelocity() - this.targetVelocity) < threshold;
  }

  public double getTargetVelocity() {
    return this.targetVelocity;
  }

  public void setPowerTurret(double pow) {
    turret.setPower(pow);
  }

  public double getTurretPosDegrees() {
    return turret.getCurrentPosition() / 2.6701388889;
  }

  public void stop() {
    this.setTargetVelocity(0);
  }
}
