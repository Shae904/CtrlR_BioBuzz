package org.firstinspires.ftc.teamcode.Auton.FarAuton;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "BLUE FAR CONFIGURABLE", group = "BLUE")
public class BlueFar15Configurable extends LinearOpMode {

  public static double[] SHOOT_BLUE = {52, 15, 110.5};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    new BaseFar15(this, robot, SHOOT_BLUE).run();
  }
}
