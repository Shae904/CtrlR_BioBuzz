package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SENSOR TEST", group = "TESTING")
public class SensorTest extends LinearOpMode {

  public double llHeight = 33.5;
  public double goalHeight = 75;


  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this);
    waitForStart();

    Limelight3A limelight = robot.limelight.limelight;

    limelight.pipelineSwitch(1);
    limelight.start();
    while (opModeIsActive()) {
      LLResult result = limelight.getLatestResult();
      List<FiducialResult> fiducials = result.getFiducialResults();
      if (result.isValid()) {
        Pose3D botPose = result.getBotpose();
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("area", result.getTa());
        telemetry.addData("distance ares est", (179.44107 * Math.pow(result.getTa(), -0.495624)));
        double angleToGoalRadians = Math.PI * ((result.getTy()) + 1.0) / 180.0;
        double distanceFromLimelightToGoal = (goalHeight - llHeight) / Math.tan(angleToGoalRadians);
        telemetry.addData("distance tangent est", distanceFromLimelightToGoal);

        telemetry.addData("botPose", botPose.toString());
        for (FiducialResult fiducial : fiducials) {
          int id = fiducial.getFiducialId(); // The ID number of the fiducial
          telemetry.addData("Tag ", id);
        }

      }
      telemetry.addData("turretPosDegrees", robot.outtake.getTurretPosDegrees());

      telemetry.update();
    }
  }
}
