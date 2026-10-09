package org.firstinspires.ftc.teamcode.Auton.FifteenBall;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "BLUE CLOSE 15", group = "BLUE")
public class BlueClose15 extends LinearOpMode {

  public static double[] SHOOT_BLUE = {60, 80, 132};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    new BaseClose15(this, robot, SHOOT_BLUE).run();
  }
}
