package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

/**
 * Shooter test: only the intake and the shooter run (no drive, the turret's rotation motor held still). The flywheel
 * runs on the robot's own velocity controller (Outtake: kS / kV / kP, voltage compensation, soft start from rest and
 * the Outtake.RECOVERY_MODE experiment chosen in Panels), as in FLYWHEEL PID TESTING's hold mode.
 *
 * <pre>
 *   circle           spin up to TARGET and hold it
 *   dpad up / down   TARGET +- TARGET_STEP (ticks/s; also while stopped, for the next spin-up)
 *   cross            stop the flywheel
 *   triangle         gate (flapper) up while held: feed into the flywheel
 *   right trigger    intake (PACED_FEED may hold it while the flywheel is below its target)
 * </pre>
 *
 * While the gate is up the opmode tracks the lowest flywheel speed it reaches; when the gate comes down it reports
 * that shot: the target, the speed when the gate opened, the lowest, how far that is below the target and when it
 * happened. Telemetry also goes to Panels, so "target", "velocity" and "lowest" can be graphed.
 */
@TeleOp(name = "SHOOTER TESTING", group = "TESTING")
@Configurable
public class ShooterTesting extends LinearOpMode {

  /** Flywheel target (ticks/s); 1220 = the shot formula at the auto shooting spots. */
  public static int TARGET = 1220;
  /** Target change per dpad press (ticks/s; the hub reports speed in steps of 20). */
  public static int TARGET_STEP = 20;

  private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

  @Override
  public void runOpMode() throws InterruptedException {
    List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : hubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    Outtake outtake = new Outtake(this);
    Intake intake = new Intake(this);   // gate starts down
    telemetry.setMsTransmissionInterval(50);

    telemetry.addLine("circle spin up | dpad up/down target | cross stop | triangle gate up | right trigger intake");
    telemetry.update();
    waitForStart();

    boolean running = false;
    boolean shooting = false;  // the gate is up
    double shotTarget = 0;
    double startVel = 0;
    double lowest = 0;
    double lowestAtMs = 0;
    int shots = 0;
    String lastShot = "none yet (hold triangle to shoot)";
    ElapsedTime sinceGate = new ElapsedTime();
    ElapsedTime loop = new ElapsedTime();

    while (opModeIsActive()) {
      for (LynxModule hub : hubs) {
        hub.clearBulkCache();
      }
      double loopMs = loop.milliseconds();
      loop.reset();

      if (gamepad1.circleWasPressed()) {
        running = true;
      } else if (gamepad1.crossWasPressed()) {
        running = false;
      }
      if (gamepad1.dpadUpWasPressed()) {
        TARGET += TARGET_STEP;
      } else if (gamepad1.dpadDownWasPressed()) {
        TARGET = Math.max(0, TARGET - TARGET_STEP);
      }
      outtake.setTargetVelocity(running ? TARGET : 0);

      boolean gateUp = gamepad1.triangle;
      intake.setFlapper(gateUp);
      outtake.setFeeding(gateUp);                       // the recovery experiments see the gate, as in the teleop
      double power = outtake.updatePIDControl();
      outtake.setPowerTurret(0);
      double intakePower = 0;
      if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
      }
      intake.setIntakePower(intakePower);

      double vel = outtake.getCurrentVelocity();        // read by updatePIDControl from this loop's bulk read
      double target = outtake.getTargetVelocity();
      if (gateUp && !shooting) {                        // the gate just opened: a new shot
        shooting = true;
        shotTarget = target;
        startVel = vel;
        lowest = vel;
        lowestAtMs = 0;
        sinceGate.reset();
      }
      if (shooting && vel < lowest) {
        lowest = vel;
        lowestAtMs = sinceGate.milliseconds();
      }
      if (!gateUp && shooting) {                        // the gate closed: report the shot
        shooting = false;
        shots++;
        lastShot = String.format("#%d: target %.0f, start %.0f, lowest %.0f (%.0f below target, %.0f below start)"
                + " at %.0f ms, gate up %.0f ms",
            shots, shotTarget, startVel, lowest, shotTarget - lowest, startVel - lowest, lowestAtMs,
            sinceGate.milliseconds());
      }

      panels.addData("Flywheel", running ? "RUNNING" : "stopped");
      panels.addData("target", target);
      panels.addData("setpoint", outtake.getSetpoint());   // the soft-start ramp / the overspeed experiment
      panels.addData("velocity", vel);
      panels.addData("error", target - vel);
      panels.addData("power", power);
      panels.addData("At target", outtake.atTarget());
      panels.addData("Gate", gateUp ? "UP (shooting)" : "down");
      panels.addData("lowest", shooting ? lowest : vel);
      panels.addData("This shot", shooting
          ? String.format("start %.0f, lowest %.0f (%.0f below target) at %.0f ms", startVel, lowest,
              shotTarget - lowest, lowestAtMs)
          : "-");
      panels.addData("Last shot", lastShot);
      panels.addData("Recovery", outtake.recoveryStatus());
      panels.addData("Battery V", outtake.getBatteryVoltage());
      panels.addData("loop ms", loopMs);
      panels.update(telemetry);
    }
    outtake.setPower(0);
    outtake.setPowerTurret(0);
    intake.setIntakePower(0);
  }
}
