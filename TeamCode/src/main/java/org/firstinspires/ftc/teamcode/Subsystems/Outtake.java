package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

  // --- PID constants (tune in Panels) ---

  // BIOBUZZ presets from the shot model (Limelight.SHOT_*): med ~2.0 m, far ~3.2 m camera -> CELL opening
  // (DECODE values were 1280 / 1640)
  public static int medSpeed = 1380;
  public static int farSpeed = 1560;

  public static int cycleSpeed = 300;

  // power = (kS + kV * target + kP * error) * NOMINAL_VOLTAGE / battery voltage
  // kS and kV come from the feedforward sweep in FLYWHEEL PID TESTING (triangle); then tune kP with its step test.
  public static double kS = 0.0;          // power to overcome friction (12 V units)
  public static double kV = 0.0004348;    // power per tick/s (12 V units)
  public static double kP = 0.003;        // power per tick/s of error
  public static boolean VOLTAGE_COMPENSATION = true; // same flywheel speed on a full or a tired battery
  public static double NOMINAL_VOLTAGE = 12.0;

  /**
   * Soft start: a jump from rest asked for full power at zero speed (full motor torque), which made the belts skip.
   * When the target rises while the flywheel is below SOFT_START_BELOW_TICKS, the speed the controller aims for climbs
   * in a straight line from the flywheel's speed to the target at SOFT_START_RATE (constant, low acceleration torque):
   * from rest 1000 in ~1.9 s, 1150 in ~2.1 s, 1560 in ~2.8 s, peak torque ~37% of stall instead of 100%. Never in an
   * @Autonomous opmode (the autos spin up exactly as before). Nothing else changes: a running flywheel (above the
   * threshold) gets new targets at once, recovery after a shot is not ramped (the aim is already at the target), and
   * lowering the target is immediate. atTarget() and getTargetVelocity() still use the real target.
   */
  public static boolean SOFT_START = true;
  public static double SOFT_START_RATE = 600;          // ticks/s gained per second
  public static double SOFT_START_BELOW_TICKS = 700;   // "starting up" below this (~30% of the 12 V free speed)
  public static double SOFT_START_MAX_LEAD = 200;      // the ramp never runs further than this ahead of the flywheel

  /**
   * EXPERIMENTS for the flywheel slowing down during a burst (first ball lands, the next ones fall short): pick ONE in
   * Panels with RECOVERY_MODE (a dropdown); OFF is the plain controller, exactly as before. Teleop and the test
   * opmodes only (never in an @Autonomous opmode), and only on a flywheel that is already up and running (not while
   * the soft start ramps, not below SOFT_START_BELOW_TICKS). Each mode has one strength number below. From mildest to
   * strongest:
   * <ul>
   *   <li>STIFFER_GAIN: kP x RECOVERY_KP_MULT while the flywheel is below its target. Smooth, mild.</li>
   *   <li>FEED_POWER: + RECOVERY_FEED_POWER while the gate is open, before any ball arrives (pushes ahead of the
   *       slow-down; the flywheel sits a little above the target while the gate is open with no ball).</li>
   *   <li>OVERSPEED_AFTER_FIRST_BALL: the first ball is shot as normal; once it is seen leaving, the flywheel aims
   *       RECOVERY_OVERSPEED_TICKS higher until the gate closes.</li>
   *   <li>KICK_ON_EACH_BALL: every time a ball is seen leaving, full power for RECOVERY_KICK_MS.</li>
   *   <li>FULL_POWER_BELOW_TARGET: full power whenever the flywheel is more than RECOVERY_BAND_TICKS below its target
   *       (bang-bang): the fastest recovery the motors can give.</li>
   *   <li>PACED_FEED: no change to the flywheel; with the gate open the intake stops pushing while the flywheel is
   *       more than RECOVERY_READY_TICKS below its target, so the next ball waits for it.</li>
   * </ul>
   * "A ball is seen leaving" = with the gate open, the flywheel's speed falls RECOVERY_SHOT_DROP_TICKS from where it
   * was (the hub reports speed in steps of 20, so 30 means two steps; lower it to 15 if balls are missed, raise it if
   * it counts balls that are not there). Telemetry "Recovery" shows the mode, the balls seen and the worst drop of
   * the burst; FLYWHEEL PID TESTING (circle, then feed with the right trigger) reports each shot's drop and recovery
   * time.
   */
  public enum RecoveryMode {
    OFF, STIFFER_GAIN, FEED_POWER, OVERSPEED_AFTER_FIRST_BALL, KICK_ON_EACH_BALL, FULL_POWER_BELOW_TARGET, PACED_FEED
  }

  public static RecoveryMode RECOVERY_MODE = RecoveryMode.OFF;
  public static double RECOVERY_KP_MULT = 4;           // STIFFER_GAIN
  public static double RECOVERY_FEED_POWER = 0.12;     // FEED_POWER (power, 12 V units)
  public static double RECOVERY_OVERSPEED_TICKS = 40;  // OVERSPEED_AFTER_FIRST_BALL
  public static double RECOVERY_KICK_MS = 80;          // KICK_ON_EACH_BALL
  public static double RECOVERY_BAND_TICKS = 20;       // FULL_POWER_BELOW_TARGET
  public static double RECOVERY_READY_TICKS = 20;      // PACED_FEED
  public static double RECOVERY_SHOT_DROP_TICKS = 30;  // ball detection (OVERSPEED_AFTER_FIRST_BALL, KICK_ON_EACH_BALL)


  public static Direction flywheel1MotorDirection = Direction.FORWARD;
  public static Direction flywheel2MotorDirection = Direction.REVERSE;


  // --- Variables ---
  private double targetVelocity = 0; // ticks/sec

  // --- Hardware ---
  public DcMotorEx flywheel1;
  public DcMotorEx flywheel2;

  public DcMotorEx turret;


  private double currentVelocity;
  private double lastPower;
  private double setpoint;                             // what the controller aims for (= the target unless ramping)
  private boolean ramping;
  /** False in @Autonomous opmodes: the autos keep the full-power spin-up and get no recovery experiment. */
  private final boolean notAutonomous;

  // shot-recovery experiments
  private boolean feeding;                             // the gate is open (teleop) / the intake is feeding (test opmode)
  private boolean atSpeedThisBurst;                    // the flywheel was at its target since the gate opened
  private double dropFrom;                             // the speed a ball's drop is measured from (recent peak)
  private double lastDetectTarget;
  private int ballsThisBurst;
  private double worstDrop;                            // furthest below the target since the gate opened (ticks/s)
  private double controlSetpoint;                      // what the controller aims for (setpoint, + the overspeed)
  private RecoveryMode activeMode = RecoveryMode.OFF;
  private final ElapsedTime sinceBall = new ElapsedTime();
  private final ElapsedTime rampTimer = new ElapsedTime();

  private final VoltageSensor battery;
  private final ElapsedTime voltageTimer = new ElapsedTime();
  private double batteryVoltage = 12.0;

  // --- Constructor ---
  public Outtake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
    flywheel2.setDirection(flywheel2MotorDirection);
    flywheel2.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel2.setMode(RunMode.RUN_WITHOUT_ENCODER);

    flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
    flywheel1.setDirection(flywheel1MotorDirection);
    flywheel1.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    flywheel1.setMode(RunMode.RUN_WITHOUT_ENCODER);

    turret = hardwareMap.get(DcMotorEx.class, "turret");
    turret.setDirection(Direction.REVERSE);
    turret.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    turret.setMode(RunMode.STOP_AND_RESET_ENCODER);
    turret.setMode(RunMode.RUN_WITHOUT_ENCODER);

    // the opmode's own class (RedHive4, RedTeleop, ...), so every @Autonomous opmode is excluded
    notAutonomous = !opMode.getClass().isAnnotationPresent(Autonomous.class);

    battery = hardwareMap.voltageSensor.iterator().next();
    readBatteryVoltage();
  }

  private void readBatteryVoltage() {
    double v = battery.getVoltage();
    if (v > 7.0) { // ignore a bad read (0 V) instead of dividing by it
      batteryVoltage = Range.clip(v, 9.0, 15.0);
    }
    voltageTimer.reset();
  }

  /** Battery voltage, re-read at most every 250 ms (a voltage read is a separate, slow hub command). */
  public double getBatteryVoltage() {
    if (voltageTimer.milliseconds() > 250) {
      readBatteryVoltage();
    }
    return batteryVoltage;
  }


  public void setPower(double pow) {
    this.flywheel1.setPower(pow);
    this.flywheel2.setPower(pow);
  }

  // --- Set target velocity ---
  public void setTargetVelocity(double targetTicksPerSec) {
    targetVelocity = Math.max(targetTicksPerSec, 0);
  }


  // --- Main PID update loop ---
  public double updatePIDControl() {
    this.currentVelocity = this.flywheel1.getVelocity(); // ticks/sec
    double dt = Math.min(rampTimer.seconds(), 0.1);      // the first call comes after the whole INIT wait
    rampTimer.reset();
    this.setpoint = softStartSetpoint(dt);

    // Shot-recovery experiment (RECOVERY_MODE): only on a flywheel that is up and running, never in the autos
    boolean running = notAutonomous && this.targetVelocity > 0 && !ramping
        && this.currentVelocity >= SOFT_START_BELOW_TICKS;
    RecoveryMode mode = running && RECOVERY_MODE != null ? RECOVERY_MODE : RecoveryMode.OFF;
    this.activeMode = mode;
    detectBalls(running);

    double aim = this.setpoint;
    if (mode == RecoveryMode.OVERSPEED_AFTER_FIRST_BALL && feeding && ballsThisBurst > 0) {
      aim += RECOVERY_OVERSPEED_TICKS;
    }
    this.controlSetpoint = aim;
    double error = aim - this.currentVelocity;

    double gain = mode == RecoveryMode.STIFFER_GAIN && error > 0 ? kP * RECOVERY_KP_MULT : kP;
    double output = (kV * aim) + (gain * error);
    if (this.targetVelocity > 0) {
      output += kS;
    }
    if (mode == RecoveryMode.FEED_POWER && feeding) {
      output += RECOVERY_FEED_POWER;
    }
    if (VOLTAGE_COMPENSATION) {
      output *= NOMINAL_VOLTAGE / getBatteryVoltage();
    }
    if (mode == RecoveryMode.FULL_POWER_BELOW_TARGET && error > RECOVERY_BAND_TICKS) {
      output = 1.0;
    }
    if (mode == RecoveryMode.KICK_ON_EACH_BALL && ballsThisBurst > 0 && sinceBall.milliseconds() < RECOVERY_KICK_MS) {
      output = 1.0;
    }

    // limit power range (a little reverse power brakes the flywheel when the target drops)
    output = Range.clip(output, -.2, 1.0);

    this.setPower(output);
    this.lastPower = output;
    return output;
  }

  /** The target, or while starting up from rest the ramp toward it (SOFT_START). */
  private double softStartSetpoint(double dt) {
    if (!SOFT_START || !notAutonomous || this.targetVelocity <= this.setpoint) {
      ramping = false;                                   // lower targets (and no ramp): at once, as before
      return this.targetVelocity;
    }
    if (!ramping) {
      if (this.currentVelocity >= SOFT_START_BELOW_TICKS) {
        return this.targetVelocity;                      // already running: at once, as before
      }
      ramping = true;                                    // starting up: ramp from where the flywheel is
      this.setpoint = Math.max(0, this.currentVelocity);
    }
    double next = Math.min(this.setpoint + SOFT_START_RATE * dt,
        Math.max(this.setpoint, this.currentVelocity + SOFT_START_MAX_LEAD));
    if (next >= this.targetVelocity) {
      ramping = false;
      return this.targetVelocity;
    }
    return next;
  }

  /** What the controller aims for now: the target, the soft-start ramp toward it, or the overspeed experiment's. */
  public double getSetpoint() {
    return this.controlSetpoint;
  }

  // ---- shot-recovery experiments ----------------------------------------------------------------------------------

  /** The gate is open / balls are being fed (teleop: the flapper is up; test opmode: the feed trigger). */
  public void setFeeding(boolean feeding) {
    if (feeding && !this.feeding) {                      // a new burst
      ballsThisBurst = 0;
      worstDrop = 0;
      atSpeedThisBurst = false;
    }
    this.feeding = feeding;
  }

  /** After a ball is seen, its drop keeps arriving for this long (the hub measures speed over 50 ms): not a new ball. */
  private static final double BALL_SETTLE_MS = 70;

  /**
   * Counts the balls of a burst from the flywheel's speed: a ball is a fall of RECOVERY_SHOT_DROP_TICKS from where the
   * speed was (its recent peak, or the target if that is lower, so readings above the target don't count), also when
   * the flywheel had not recovered from the ball before.
   */
  private void detectBalls(boolean running) {
    double target = this.targetVelocity;
    double v = this.currentVelocity;
    if (!feeding || !running) {
      dropFrom = v;                                      // no burst: just follow the speed
      lastDetectTarget = target;
      return;
    }
    if (target < lastDetectTarget) {
      dropFrom -= lastDetectTarget - target;             // slowing down for a lower target is not a ball
    }
    lastDetectTarget = target;
    double below = target - v;
    if (atSpeedThisBurst || below <= RECOVERY_SHOT_DROP_TICKS * 2 / 3) {
      atSpeedThisBurst = true;
      worstDrop = Math.max(worstDrop, below);
    }
    if (ballsThisBurst > 0 && sinceBall.milliseconds() < BALL_SETTLE_MS) {
      dropFrom = v;                                      // still the same ball
      return;
    }
    dropFrom = Math.max(dropFrom, v);
    if (v <= Math.min(dropFrom, target) - RECOVERY_SHOT_DROP_TICKS) {
      ballsThisBurst++;
      sinceBall.reset();
      dropFrom = v;
    }
  }

  /**
   * PACED_FEED: false while the gate is open and the flywheel is more than RECOVERY_READY_TICKS below its target (the
   * caller then holds the intake); true in every other mode and in the autos.
   */
  public boolean feedAllowed() {
    return !(notAutonomous && RECOVERY_MODE == RecoveryMode.PACED_FEED && feeding && this.targetVelocity > 0
        && this.targetVelocity - this.currentVelocity > RECOVERY_READY_TICKS);
  }

  /** For telemetry: the experiment in use, the balls seen this burst and the worst drop below the target. */
  public String recoveryStatus() {
    String mode = !notAutonomous ? "off in autos"
        : RECOVERY_MODE == RecoveryMode.PACED_FEED ? "PACED_FEED" + (feedAllowed() ? "" : " (holding the intake)")
        : RECOVERY_MODE == activeMode || RECOVERY_MODE == null ? String.valueOf(activeMode)
        : RECOVERY_MODE + " (waiting: flywheel not up to speed)";
    return String.format("%s | balls seen %d | worst drop %.0f ticks/s", mode, ballsThisBurst, worstDrop);
  }

  public double getLastPower() {
    return this.lastPower;
  }

  public double getCurrentVelocity() {
    return this.currentVelocity;
  }

  public boolean atTarget() {
    return atTarget(40);
  }

  public boolean atTarget(double threshold) {
    return Math.abs(this.getCurrentVelocity() - this.targetVelocity) < threshold;
  }

  public double getTargetVelocity() {
    return this.targetVelocity;
  }

  public void setPowerTurret(double pow) {
    turret.setPower(pow);
  }

  public double getTurretPosDegrees() {
    return turret.getCurrentPosition() / 2.6701388889;
  }

  public void stop() {
    this.setTargetVelocity(0);
  }
}
