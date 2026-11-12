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
 * Shooter test: only the intake and the shooter run (no drive, the turret's rotation motor held still).
 *
 * <pre>
 *   dpad up / down   shooter (flywheel) power +- POWER_STEP, open loop (no PID: the raw spin-down shows)
 *   cross            shooter power 0
 *   triangle         gate (flapper) up while held: feed into the flywheel
 *   right trigger    intake
 * </pre>
 *
 * While the gate is up the opmode tracks the lowest flywheel speed it reaches; when the gate comes down it reports
 * that shot: the speed when the gate opened, the lowest, the drop and when it happened. Rising power is ramped at
 * POWER_RAMP_PER_S (a jump from rest skips the belts); lowering it is immediate. Telemetry also goes to Panels, so
 * "velocity" and "lowest" can be graphed.
 */
@TeleOp(name = "SHOOTER TESTING", group = "TESTING")
@Configurable
public class ShooterTesting extends LinearOpMode {

  /** Shooter power change per dpad press. */
  public static double POWER_STEP = 0.05;
  /** Rising power is applied at most this much faster per second (the PID test's sweep uses 0.3 for the belts). */
  public static double POWER_RAMP_PER_S = 0.3;

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

    telemetry.addLine("dpad up/down shooter power | cross power 0 | triangle gate up | right trigger intake");
    telemetry.update();
    waitForStart();

    double power = 0;          // asked for with the dpad
    double applied = 0;        // sent to the motors (ramped up)
    boolean shooting = false;  // the gate is up
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
      double dt = Math.min(loop.seconds(), 0.1);
      loop.reset();

      if (gamepad1.dpadUpWasPressed()) {
        power = Math.min(1, power + POWER_STEP);
      }
      if (gamepad1.dpadDownWasPressed()) {
        power = Math.max(0, power - POWER_STEP);
      }
      if (gamepad1.crossWasPressed()) {
        power = 0;
      }
      applied = power > applied ? Math.min(power, applied + POWER_RAMP_PER_S * dt) : power;
      outtake.setPower(applied);
      outtake.setPowerTurret(0);

      boolean gateUp = gamepad1.triangle;
      intake.setFlapper(gateUp);
      intake.setIntakePower(gamepad1.right_trigger);

      double vel = outtake.flywheel1.getVelocity();   // from this loop's bulk read
      if (gateUp && !shooting) {                       // the gate just opened: a new shot
        shooting = true;
        startVel = vel;
        lowest = vel;
        lowestAtMs = 0;
        sinceGate.reset();
      }
      if (shooting && vel < lowest) {
        lowest = vel;
        lowestAtMs = sinceGate.milliseconds();
      }
      if (!gateUp && shooting) {                       // the gate closed: report the shot
        shooting = false;
        shots++;
        lastShot = String.format("#%d: start %.0f, lowest %.0f (drop %.0f) at %.0f ms, gate up %.0f ms, power %.2f",
            shots, startVel, lowest, startVel - lowest, lowestAtMs, sinceGate.milliseconds(), applied);
      }

      panels.addData("Power (dpad)", String.format("%.2f", power));
      panels.addData("Power applied", applied);
      panels.addData("velocity", vel);
      panels.addData("Gate", gateUp ? "UP (shooting)" : "down");
      panels.addData("lowest", shooting ? lowest : vel);
      panels.addData("This shot", shooting
          ? String.format("start %.0f, lowest %.0f (drop %.0f) at %.0f ms", startVel, lowest, startVel - lowest,
              lowestAtMs)
          : "-");
      panels.addData("Last shot", lastShot);
      panels.addData("Battery V", outtake.getBatteryVoltage());
      panels.update(telemetry);
    }
    outtake.setPower(0);
    outtake.setPowerTurret(0);
    intake.setIntakePower(0);
  }
}
