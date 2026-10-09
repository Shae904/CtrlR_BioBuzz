package org.firstinspires.ftc.teamcode.Auton.FifteenBall;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED CLOSE 15", group = "RED")
public class RedClose15 extends LinearOpMode {

  public static double[] SHOOT_RED = {87, 80, 48};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    new BaseClose15(this, robot, SHOOT_RED).run();
  }
}
