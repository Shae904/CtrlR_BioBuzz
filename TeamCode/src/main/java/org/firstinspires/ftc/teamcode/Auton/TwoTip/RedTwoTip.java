package org.firstinspires.ftc.teamcode.Auton.TwoTip;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED 2 TIP", group = "MAIN")
public class RedTwoTip extends LinearOpMode {

  /** x in, y in, heading deg: the same start as RED HIVE 4. */
  public static double[] START = {46.0, 7.36, 90};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    robot.follower.setPose(new Pose(START[0], START[1], Math.toRadians(START[2])));
    new BaseTwoTip(this, robot).run();
  }
}
