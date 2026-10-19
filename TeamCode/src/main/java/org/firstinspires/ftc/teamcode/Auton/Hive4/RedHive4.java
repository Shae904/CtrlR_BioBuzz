package org.firstinspires.ftc.teamcode.Auton.Hive4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED HIVE 4", group = "MAIN")
public class RedHive4 extends LinearOpMode {

  /**
   * x in, y in, heading deg (Pedro field frame: origin at the red/audience corner): back against the audience wall,
   * robot centre 46 in from the red wall (tile column B), facing the far wall (G304).
   */
  public static double[] START = {46.0, 7.36, 90};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    robot.follower.setPose(new Pose(START[0], START[1], Math.toRadians(START[2])));
    new BaseHive4(this, robot).run();
  }
}
