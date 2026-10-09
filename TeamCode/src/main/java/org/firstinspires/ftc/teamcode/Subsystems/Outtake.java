package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

  // --- PID constants (tune in Panels) ---

  public static int medSpeed = 1280; //1380
  public static int farSpeed = 1640;

  public static int cycleSpeed = 300;
  public static double kP = 0.003;
  public static double kV = 0.0004348;


  public static Direction flywheel1MotorDirection = Direction.FORWARD;
  public static Direction flywheel2MotorDirection = Direction.REVERSE;


  // --- Variables ---
  private double targetVelocity = 0; // ticks/sec

  // --- Hardware ---
  public DcMotorEx flywheel1;
  public DcMotorEx flywheel2;

  public DcMotorEx turret;


  private double currentVelocity;

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

    // limit power range
    output = Range.clip(output, -.2, 1.0);

    this.setPower(output);
    return output;
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
