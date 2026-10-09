package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Configurable
@TeleOp(name = "SERVO TESTING", group = "TESTING")
public class ServoTest extends LinearOpMode {

  Robot robot;
  public static double OUTTAKE_TEST_POS = Intake.FLAPPER_LOCKED;
  public static double CYCLER_TEST_POS = Intake.CYCLER_LOCKED;

  public static double TILT_A_TEST_POS = Robot.TILT_A_FOLDED;
  public static double TILT_B_TEST_POS = Robot.TILT_B_FOLDED;


  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // LOOP
    while (opModeIsActive()) {
      robot.intake.setFlapperPos(OUTTAKE_TEST_POS);
      robot.intake.setCyclerPos(CYCLER_TEST_POS);

      robot.setTiltPos(TILT_A_TEST_POS, TILT_B_TEST_POS);

      if (gamepad1.a) {
        robot.fl.setPower(1);
      } else {
        robot.fl.setPower(0);
      }
      if (gamepad1.b) {
        robot.bl.setPower(1);
      } else {
        robot.bl.setPower(0);
      }
      if (gamepad1.x) {
        robot.fr.setPower(1);
      } else {
        robot.fr.setPower(0);
      }
      if (gamepad1.y) {
        robot.br.setPower(1);
      } else {
        robot.br.setPower(0);
      }

      if (gamepad1.right_bumper) {
        robot.outtake.setPowerTurret(1);
      } else {
        robot.outtake.setPowerTurret(0);
      }

    }
  }
}
