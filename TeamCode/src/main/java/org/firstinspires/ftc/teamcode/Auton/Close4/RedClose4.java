package org.firstinspires.ftc.teamcode.Auton.Close4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "CLOSE RED 4", group = "MAIN")
public class RedClose4 extends LinearOpMode {

  /**
   * x in, y in, heading deg: the red/rear corner beside the LOADING ZONE, facing the blue wall, back against the red
   * wall (robot centre 7.36 in off it) and left side against the rear wall (8.37 in off it). The turret points right.
   */
  public static double[] START = {7.36, 132.68, 0};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    robot.follower.setPose(new Pose(START[0], START[1], Math.toRadians(START[2])));
    new BaseClose4(this, robot).run();
  }
}
