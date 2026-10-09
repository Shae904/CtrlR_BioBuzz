package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.teamcode.AllianceColor;

@Configurable
public class Limelight {

  public static double AIM_Kp = 0.02;
  public static double AIM_Ki = 0;
  public static double AIM_Kd = 0.002;
  public static double AIM_Ks = 0.04;

  public static double AIM_DEADBAND = .3;

  public static double AIM_RGB_THRESHOLD = 4;


  public final Limelight3A limelight;
  private final AllianceColor currentColor;
  private final ElapsedTime aimTimer = new ElapsedTime();

  public final Servo rgb;

  private LLResult currentGoal;

  private double lastCalculatedVel = Outtake.medSpeed;
  private double aimIntegral = 0;
  private double aimLastError = 0;

  private double vPerpendicular;
  private double vParallel;

  public double target;

  public double distance;
  public double error;

  public double turretPosition;
  private double lastTarget;

  public Limelight(LinearOpMode opMode, AllianceColor color) { // Constructor
    HardwareMap hardwareMap = opMode.hardwareMap;
    this.currentColor = color;
    this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    this.limelight.start();
    this.limelight.pipelineSwitch(this.currentColor.getLLPipelineTeleOP());

    this.lastTarget = this.currentColor.getSteadyState();
    this.turretPosition = this.currentColor.getSteadyState();

    this.rgb = hardwareMap.servo.get("rgb");
    this.rgb.setPosition(.5);
  }

  public void setTarget(double pos) {
    this.lastTarget = pos;
  }


  public void updateAim(double xVelocity, double yVelocity) {
    // Update goal
    this.currentGoal = this.limelight.getLatestResult();

    // Update Velocities and Angles
    double txRad = Math.toRadians(currentGoal.getTx());

    double turretRad = Math.toRadians(turretPosition + 180);
    // Total angle = turret heading + tx offset from turret center
    double totalAngleRad = turretRad + txRad;

    this.vPerpendicular = xVelocity * Math.cos(totalAngleRad)
        - yVelocity * Math.sin(totalAngleRad);

    this.vParallel = xVelocity * Math.sin(totalAngleRad)
        + yVelocity * Math.cos(totalAngleRad); // figure this out

    if (currentGoal != null && currentGoal.isValid()) {
      this.distance = (((41.275) / Math.tan((Math.toRadians(currentGoal.getTy() + 1.0)))) / 100.0);
    }

  }

  /**
   * Calculates the target velocity for the shooter based on the current teleop goal tag reading
   *
   * @return the target power for the shooter, or the last calculated power if no valid reading
   */
  public double calculateTargetVelocity() {
    double calculatedVel;
    if (this.hasValidTarget()) {

      calculatedVel = (20.0 * (Math.round(
          (((distance * Math.pow(0.243301244553 * distance - 0.173469387755, -0.5)) / 0.0025344670037)
              - (vParallel * 93 * Math.cos(Math.toRadians(50)))) // 253
              / 20.0))) - 100; //80

      lastCalculatedVel = calculatedVel;

      if (distance > 2.5) {
        if (this.currentColor == AllianceColor.BLUE) {
          return calculatedVel - 20;
        }
        return calculatedVel + 20;
      }

      return calculatedVel;
    }
    return lastCalculatedVel;
  }

  private double calculateLeadAngleDegrees() {
    return Math.toDegrees(Math.atan(
        (vPerpendicular * Math.sqrt((2 * ((1.192 * distance) - .85)) / 9.46))

            / distance

    ));
  }

  public double calculateError() {
    return currentGoal.getTx() - (currentColor.getAimPose() + this.calculateLeadAngleDegrees());
  }

  public boolean hasValidTarget() {
    return currentGoal != null && currentGoal.isValid();
  }

  public void updateTarget(double turretPos, boolean shouldAim, AllianceColor color) {
    this.turretPosition = turretPos + color.getSteadyState();

    if (this.hasValidTarget() && shouldAim) {
      double rawTarget = this.turretPosition + this.calculateError();
      // wrap
      rawTarget = ((rawTarget % 360) + 360) % 360;

      if (Math.abs(error) < 45) {
        target = rawTarget;
        lastTarget = target;
      }

    } else if (shouldAim && !this.hasValidTarget()) {
      target = lastTarget;

    } else if (!shouldAim) {
      target = color.getSteadyState();
    }

    error = target - this.turretPosition;
  }


  public double updateAimPID() { // returns the turn power from pid for autoaiming
    double dt = aimTimer.seconds();
    aimTimer.reset();

    // Integral
    aimIntegral += error * dt;

    // Derivative
    double derivative = (error - aimLastError) / dt;
    derivative = Range.clip(derivative, -50, 50); // prevent explosion on large error jumps
    aimLastError = error;

    // PID Output

    if (Math.abs(error) < AIM_DEADBAND) {
      aimIntegral = 0;
      return 0;
    }

    if (target == this.currentColor.getSteadyState()) {
      rgb.setPosition(.277);
    } else if (Math.abs(error) < AIM_RGB_THRESHOLD && this.hasValidTarget()) {
      rgb.setPosition(.5);
    } else if (Math.abs(error) > AIM_RGB_THRESHOLD && this.hasValidTarget()) {
      rgb.setPosition(0.277);
    } else if (Math.abs(error) < AIM_RGB_THRESHOLD && !this.hasValidTarget()) {
      rgb.setPosition(.611);
    } else {
      rgb.setPosition(0);
    }

    double output = AIM_Kp * error
        + AIM_Ki * aimIntegral
        + AIM_Kd * derivative
        + AIM_Ks * Math.signum(error);

    // Clamp for safety
    output = Range.clip(output, -1, 1);
    return output;   // return turn power
  }


  public int getPatternIdAuto() { // only for auto just returns the tag id for patterns
    this.limelight.pipelineSwitch(0);
    LLResult result = this.limelight.getLatestResult();
    List<FiducialResult> fiducials = result.getFiducialResults();
    if (result.isValid()) {
      for (FiducialResult fiducial : fiducials) {
        return fiducial.getFiducialId();
      }
    }
    return 21;
  }

  public void setPipeline(AllianceColor color) {
    this.limelight.pipelineSwitch(color.getLLPipelineTeleOP());
  }

}
