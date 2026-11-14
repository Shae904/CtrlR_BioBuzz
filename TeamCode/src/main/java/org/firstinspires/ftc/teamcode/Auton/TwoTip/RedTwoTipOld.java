package org.firstinspires.ftc.teamcode.Auton.TwoTip;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

/** The previous RED 2 TIP route (side spots, around the outside of the HIVE), kept as a fallback. */
@Autonomous(name = "RED 2 TIP OLD", group = "MAIN")
public class RedTwoTipOld extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED);
    robot.follower.setPose(new Pose(RedTwoTip.START[0], RedTwoTip.START[1], Math.toRadians(RedTwoTip.START[2])));
    new BaseTwoTip(this, robot, true).run();
  }
}
