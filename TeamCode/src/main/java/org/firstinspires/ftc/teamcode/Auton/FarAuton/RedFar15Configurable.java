package org.firstinspires.ftc.teamcode.Auton.FarAuton;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED FAR CONFIGURABLE", group = "RED")
public class RedFar15Configurable extends LinearOpMode {

  public static double[] SHOOT_RED = {90, 15, 67};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    new BaseFar15(this, robot, SHOOT_RED).run();
  }
}
