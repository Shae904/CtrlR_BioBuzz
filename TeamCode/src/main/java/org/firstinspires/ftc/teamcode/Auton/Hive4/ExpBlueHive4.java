package org.firstinspires.ftc.teamcode.Auton.Hive4;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Experimental BLUE HIVE 4: the same routine and start as BLUE HIVE 4, but it shoots from SHOOT_RED's spot (13.2, 20
 * rotated), where the robot stays 0.5 in off the blue wall, instead of BLUE HIVE 4's comp 1 spot, where its corner is
 * 0.64 in into the wall.
 */
@Configurable
@Autonomous(name = "EXP_BLUE HIVE 4", group = "MAIN")
public class ExpBlueHive4 extends LinearOpMode {

  /** x in, y in, heading deg: the same start as BLUE HIVE 4. */
  public static double[] START = {95.05, 133.69, 270};

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE);
    robot.follower.setPose(new Pose(START[0], START[1], Math.toRadians(START[2])));
    new BaseHive4(this, robot, true).run();
  }
}
