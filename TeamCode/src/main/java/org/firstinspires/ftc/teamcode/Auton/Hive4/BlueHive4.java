package org.firstinspires.ftc.teamcode.Auton.Hive4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "BLUE HIVE 4", group = "MAIN")
public class BlueHive4 extends LinearOpMode {

  /**
   * x in, y in, heading deg: the RED start rotated 180 deg about the field centre: back against the rear wall, robot
   * centre 46 in from the blue wall (tile column E), facing the audience.
   */
  public static double[] START = {95.05, 133.69, 270};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    robot.follower.setPose(new Pose(START[0], START[1], Math.toRadians(START[2])));
    new BaseHive4(this, robot).run();
  }
}
