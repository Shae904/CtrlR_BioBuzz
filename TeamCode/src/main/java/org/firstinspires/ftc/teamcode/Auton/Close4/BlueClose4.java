package org.firstinspires.ftc.teamcode.Auton.Close4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "CLOSE BLUE 4", group = "MAIN")
public class BlueClose4 extends LinearOpMode {

  /**
   * x in, y in, heading deg: the RED start rotated 180 deg about the field centre: the blue/audience corner beside the
   * LOADING ZONE, facing the red wall, back against the blue wall and left side against the audience wall. The turret
   * points right.
   */
  public static double[] START = {133.69, 8.37, 180};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    robot.follower.setPose(new Pose(START[0], START[1], Math.toRadians(START[2])));
    new BaseClose4(this, robot).run();
  }
}
