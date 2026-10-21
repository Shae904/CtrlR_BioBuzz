package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * Pedro 3 AutoTune entry points. Sloth's TunerScanner (com.pedropathing:tuning) finds every static, zero-argument
 * method annotated @Tuner that returns Procedure, and lists it at http://192.168.43.1:10158 (no @TeleOp needed:
 * each step runs as a temporary "Pedro Tuning" utility opmode).
 *
 * Each @Tuner names itself: tuning 1.0.3-beta.1's scanner registers annotation.name() as-is (it no longer falls back
 * to the procedure's own name), so an unnamed @Tuner would show up blank on the tuning page.
 */
public class Tuning {
  @Tuner(name = "Mecanum Tuner")
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  @Tuner(name = "Pinpoint Tuner")
  public static Procedure pinpointTuner() {
    return new PinpointTuner();
  }

  @Tuner(name = "Foresight Tuner")
  public static Procedure foresightTuner() {
    return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
  }

  @Tuner(name = "Tests")
  public static Procedure tests() {
    return new Tests(Constants::createDrivetrain, Constants::createLocalizer, () -> new Foresight(Constants.foresightConfig));
  }
}
