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
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlapperState;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

/**
 * Flywheel velocity-control tuning. Watch the graph in Panels (http://192.168.43.1:8001): target, velocity, error,
 * power. Gains are the live Panels fields Outtake.kS / kV / kP; copy the final values into Outtake.java.
 *
 * <pre>
 *   TRIANGLE  feedforward sweep (~40 s): open-loop power steps, fits kS and kV (12 V units) and applies them
 *   SQUARE    step test: target alternates LOW_TARGET / HIGH_TARGET every STEP_PERIOD_S  (tune kP here)
 *   CIRCLE    hold HOLD_TARGET (dpad up/down = +-20); feed POLLEN with right trigger to see shot recovery
 *   CROSS     stop
 *   triggers  intake (right = feed into the flywheel, left = reverse), in any mode
 * </pre>
 *
 * Only the flywheel and the intake run; the drivetrain never moves.
 */
@TeleOp(name = "FLYWHEEL PID TESTING", group = "TESTING")
@Configurable
public class FlywheelPIDTesting extends LinearOpMode {

  // Step test (defaults = the shot presets, Outtake.medSpeed / farSpeed)
  public static int LOW_TARGET = 1380;
  public static int HIGH_TARGET = 1560;
  public static double STEP_PERIOD_S = 2.5;
  // Hold / shot recovery
  public static int HOLD_TARGET = 1460;
  public static double BAND = 40;           // "at target" band in ticks/s (same as Outtake.atTarget())
  public static double SHOT_DIP = 20;       // a drop this far below the target counts as a shot
  // Feedforward sweep
  public static double FF_MIN_POWER = 0.30;
  public static double FF_MAX_POWER = 0.80;
  public static int FF_STEPS = 6;
  public static double FF_DWELL_S = 5.0;    // per step: the open-loop time constant is ~1.4 s (CAD inertia)
  public static double FF_SAMPLE_S = 1.0;   // averaged at the end of each step
  public static boolean FF_APPLY = true;    // write the fitted kS / kV into Outtake for this session

  private enum Mode { IDLE, SWEEP, STEP, HOLD }

  private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
  private Outtake outtake;
  private Mode mode = Mode.IDLE;

  // sweep state
  private int sweepSteps;
  private int sweepIndex;
  private final ElapsedTime sweepTimer = new ElapsedTime();
  private double sumVel, sumVolt;
  private int samples;
  private double[] sweepVel, sweepPower12;
  private String sweepResult = "not run (press TRIANGLE)";

  // step test state
  private final ElapsedTime stepTimer = new ElapsedTime();
  private boolean stepHigh;
  private int stepDir;
  private double riseS, overshoot, lastOutsideS;
  private String upResult = "-", downResult = "-";

  // hold state
  private final ElapsedTime settled = new ElapsedTime();
  private final ElapsedTime dipTimer = new ElapsedTime();
  private boolean inDip;
  private double dipMax;
  private int shots;
  private String shotResult = "-";

  @Override
  public void runOpMode() throws InterruptedException {
    List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : hubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    outtake = new Outtake(this);
    Intake intake = new Intake(this);
    telemetry.setMsTransmissionInterval(50);

    telemetry.addLine("TRIANGLE sweep (kS/kV) | SQUARE step test (kP) | CIRCLE hold + feed | CROSS stop");
    telemetry.update();
    waitForStart();
    intake.setCyclePosition(FlapperState.SHOOT);

    ElapsedTime loop = new ElapsedTime();
    while (opModeIsActive()) {
      for (LynxModule hub : hubs) {
        hub.clearBulkCache();
      }
      double loopMs = loop.milliseconds();
      loop.reset();

      if (gamepad1.triangleWasPressed()) {
        startSweep();
      } else if (gamepad1.squareWasPressed()) {
        startStep();
      } else if (gamepad1.circleWasPressed()) {
        mode = Mode.HOLD;
        inDip = false;
        settled.reset();
      } else if (gamepad1.crossWasPressed()) {
        mode = Mode.IDLE;
      }
      if (gamepad1.dpadUpWasPressed()) {
        HOLD_TARGET += 20;
        settled.reset(); // a target change is not a shot
        inDip = false;
      } else if (gamepad1.dpadDownWasPressed()) {
        HOLD_TARGET = Math.max(0, HOLD_TARGET - 20);
        settled.reset();
        inDip = false;
      }

      double power;
      switch (mode) {
        case SWEEP:
          power = updateSweep();
          break;
        case STEP:
          updateStep();
          power = outtake.updatePIDControl();
          break;
        case HOLD:
          outtake.setTargetVelocity(HOLD_TARGET);
          power = outtake.updatePIDControl();
          updateHold();
          break;
        default:
          outtake.setTargetVelocity(0);
          power = outtake.updatePIDControl();
          break;
      }

      intake.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);

      double vel = outtake.flywheel1.getVelocity(); // from this loop's bulk read (no extra hub call)
      double target = mode == Mode.SWEEP ? 0 : outtake.getTargetVelocity();
      panels.addData("mode", mode);
      panels.addData("target", target);
      panels.addData("velocity", vel);
      panels.addData("error", mode == Mode.SWEEP ? 0 : target - vel);
      panels.addData("power", power);
      panels.addData("battery V", outtake.getBatteryVoltage());
      panels.addData("loop ms", loopMs);
      panels.addData("gains", String.format("kS %.4f  kV %.7f  kP %.4f  Vcomp %s",
          Outtake.kS, Outtake.kV, Outtake.kP, Outtake.VOLTAGE_COMPENSATION));
      panels.addData("sweep", sweepResult);
      panels.addData("step up", upResult);
      panels.addData("step down", downResult);
      panels.addData("shots", shotResult);
      panels.update(telemetry);
    }
    outtake.setPower(0);
    intake.setIntakePower(0);
  }

  // ---------------- feedforward sweep ----------------

  private void startSweep() {
    mode = Mode.SWEEP;
    sweepSteps = Math.max(2, FF_STEPS);
    sweepIndex = 0;
    sweepVel = new double[sweepSteps];
    sweepPower12 = new double[sweepSteps];
    sumVel = 0;
    sumVolt = 0;
    samples = 0;
    sweepTimer.reset();
    sweepResult = "running...";
  }

  private double sweepPower(int i) {
    return FF_MIN_POWER + (FF_MAX_POWER - FF_MIN_POWER) * i / (sweepSteps - 1);
  }

  /** Open loop: holds each power until the flywheel settles, records (velocity, power in 12 V units). */
  private double updateSweep() {
    double power = sweepPower(sweepIndex);
    outtake.setPower(power);
    double vel = outtake.flywheel1.getVelocity();
    double dwell = FF_DWELL_S + (sweepIndex == 0 ? 2.0 : 0.0); // the first step starts from rest
    double t = sweepTimer.seconds();
    if (t > dwell - FF_SAMPLE_S) {
      sumVel += vel;
      sumVolt += outtake.getBatteryVoltage();
      samples++;
    }
    sweepResult = String.format("step %d/%d  power %.2f  velocity %.0f", sweepIndex + 1, sweepSteps, power, vel);
    if (t >= dwell && samples > 0) {
      double volt = sumVolt / samples;
      sweepVel[sweepIndex] = sumVel / samples;
      sweepPower12[sweepIndex] = power * volt / Outtake.NOMINAL_VOLTAGE;
      sweepIndex++;
      sumVel = 0;
      sumVolt = 0;
      samples = 0;
      sweepTimer.reset();
      if (sweepIndex >= sweepSteps) {
        finishSweep();
        mode = Mode.IDLE;
        outtake.setTargetVelocity(0);
        return outtake.updatePIDControl();
      }
    }
    return power;
  }

  private void finishSweep() {
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    boolean negative = false;
    for (int i = 0; i < sweepSteps; i++) {
      double x = sweepVel[i], y = sweepPower12[i];
      if (x < -100) {
        negative = true;
      }
      if (x < 100) {
        continue; // not spinning at this power
      }
      n++;
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
    }
    if (negative) {
      sweepResult = "FAILED: flywheel1 velocity is negative under positive power. Flip flywheel1MotorDirection AND"
          + " flywheel2MotorDirection in Outtake, or the PID will run away.";
      return;
    }
    double den = n * sxx - sx * sx;
    if (n < 2 || Math.abs(den) < 1e-9) {
      sweepResult = "FAILED: flywheel did not spin (velocity < 100 at every power). Check flywheel1 / encoder.";
      return;
    }
    double kV = (n * sxy - sx * sy) / den;
    double kS = (sy - kV * sx) / n;
    double worst = 0;
    for (int i = 0; i < sweepSteps; i++) {
      if (sweepVel[i] >= 100) {
        worst = Math.max(worst, Math.abs(sweepPower12[i] - (kS + kV * sweepVel[i])) / kV);
      }
    }
    if (FF_APPLY) {
      Outtake.kS = kS;
      Outtake.kV = kV;
    }
    sweepResult = String.format("kS %.4f  kV %.7f  (fit error %.0f ticks/s, 12 V free speed %.0f)%s",
        kS, kV, worst, (1 - kS) / kV, FF_APPLY ? "  APPLIED" : "");
  }

  // ---------------- step test ----------------

  private void startStep() {
    mode = Mode.STEP;
    stepHigh = true;
    beginStep(HIGH_TARGET);
  }

  private void beginStep(double target) {
    stepDir = target >= outtake.getCurrentVelocity() ? 1 : -1;
    outtake.setTargetVelocity(target);
    stepTimer.reset();
    riseS = -1;
    overshoot = 0;
    lastOutsideS = 0;
  }

  private void updateStep() {
    if (stepTimer.seconds() >= STEP_PERIOD_S) {
      String r = riseS < 0
          ? "never got within BAND"
          : String.format("rise %.0f ms  overshoot %.0f  settled %.0f ms", riseS * 1000, overshoot, lastOutsideS * 1000);
      if (stepHigh) {
        upResult = r;
      } else {
        downResult = r;
      }
      stepHigh = !stepHigh;
      beginStep(stepHigh ? HIGH_TARGET : LOW_TARGET);
    }
    double t = stepTimer.seconds();
    double err = outtake.getTargetVelocity() - outtake.getCurrentVelocity();
    if (riseS < 0 && Math.abs(err) < BAND) {
      riseS = t;
    }
    if (riseS >= 0) {
      overshoot = Math.max(overshoot, -stepDir * err); // beyond the target in the step direction
    }
    if (Math.abs(err) >= BAND) {
      lastOutsideS = t;
    }
  }

  // ---------------- hold / shot recovery ----------------

  private void updateHold() {
    double err = outtake.getTargetVelocity() - outtake.getCurrentVelocity(); // > 0: below target
    if (Math.abs(err) >= BAND && !inDip) {
      settled.reset();
    }
    if (!inDip && err > SHOT_DIP && settled.seconds() > 0.3) {
      inDip = true;
      dipMax = err;
      dipTimer.reset();
    } else if (inDip) {
      dipMax = Math.max(dipMax, err);
      if (err < SHOT_DIP / 2) {
        shots++;
        shotResult = String.format("#%d: dropped %.0f ticks/s, recovered in %.0f ms", shots, dipMax,
            dipTimer.milliseconds());
        inDip = false;
        settled.reset();
      }
    }
  }
}
