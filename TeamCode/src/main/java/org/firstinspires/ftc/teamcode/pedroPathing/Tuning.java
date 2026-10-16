package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ThreeWheelTuner;

/**
 * Pedro 3 AutoTune entry points. Sloth's TunerScanner (com.pedropathing:tuning) finds every static, zero-argument
 * method annotated @Tuner that returns Procedure, and lists it at http://192.168.43.1:10158 (no @TeleOp needed:
 * each step runs as a temporary "Pedro Tuning" utility opmode).
 */
public class Tuning {
  @Tuner
  public static Procedure mecanumTuner() {
    return new MecanumTuner();
  }

  @Tuner
  public static Procedure threeWheelTuner() {
    return new ThreeWheelTuner();
  }

  @Tuner
  public static Procedure foresightTuner() {
    return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
  }

  @Tuner
  public static Procedure tests() {
    return new Tests(Constants::createDrivetrain, Constants::createLocalizer, () -> new Foresight(Constants.foresightConfig));
  }
}
