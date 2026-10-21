package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0.2 constants for the BIOBUZZ robot (migrated from the 2.0.6 FollowerConstants / MecanumConstants /
 * PathConstraints; the three-wheel localizer was replaced by a goBILDA Pinpoint with two pods). Values marked "2.x" are
 * carried over unchanged; values marked "ESTIMATE" have no 2.x equivalent and must be replaced by the AutoTune
 * Foresight Tuner's generated block before any path is run.
 */
public class Constants {

  // ---- Drivetrain (2.x MecanumConstants) -------------------------------------------------------------------------
  public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
    c.frontLeftName.set("fl");                                 // 2.x leftFrontMotorName("fl")
    c.backLeftName.set("bl");                                  // 2.x leftRearMotorName("bl")
    c.frontRightName.set("fr");                                // 2.x rightFrontMotorName("fr")
    c.backRightName.set("br");                                 // 2.x rightRearMotorName("br")
    c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE); // 2.x leftFrontMotorDirection
    c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);  // 2.x leftRearMotorDirection
    c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);// 2.x rightFrontMotorDirection
    c.backRightDirection.set(DcMotorSimple.Direction.FORWARD); // 2.x rightRearMotorDirection
    c.manualBrakeMode.set(true);   // keep true: an idle follower.update() calls stop(), which sets BRAKE here; with
                                   // false it would switch the teleop drive motors to FLOAT every loop (Robot sets BRAKE)
    c.powerThreshold.set(0.01);    // 3.x default; CachedMotor skips writes smaller than this
    // 2.x maxPower(1): no 3.x equivalent (Mecanum always normalises to |power| <= 1); cap speed with
    //   foresightConfig.maxPathSpeed instead.
    // 2.x xVelocity / yVelocity: moved to foresightConfig.maxAchievable{Forward,Strafe}Velocity below.
  });

  // ---- Localizer: goBILDA Pinpoint with two pods (the left parallel pod and the strafe pod) -------------------------
  // The Pinpoint integrates the two pods with its own IMU and reports a pose over I2C ("pinpoint" in the robot
  // configuration, on an I2C port of the Control Hub). Offsets are where each pod's wheel sits relative to the robot's
  // centre of rotation: xPodOffset = the forward (X) pod's sideways position (left positive), yPodOffset = the strafe
  // (Y) pod's forward position (forward positive). Starting values from the CAD / the old ThreeWheel constants (left
  // pod 7.5 in left of centre, strafe pod 2 in behind it); directions assume both pods count forward / left.
  // Tune all four with AutoTune's "Pinpoint Tuner" (pedroPathing/procedures/PinpointTuner): push forward, push left,
  // spin 180 deg CCW, then paste its block here.
  public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);   // 32 mm wheel, 2000 CPR
    c.xPodOffset.set(7.5);
    c.yPodOffset.set(-2.0);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
  });

  // ---- Algorithm (2.x FollowerConstants + PathConstraints) --------------------------------------------------------
  public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
    // Same physical quantities, measured the same way by the 2.x and 3.x tuners:
    c.maxAchievableForwardVelocity.set(80.21);      // 2.x MecanumConstants.xVelocity (in/s)
    c.maxAchievableStrafeVelocity.set(65.9686);     // 2.x MecanumConstants.yVelocity (in/s)
    c.naturalForwardDeceleration.set(57.3022);      // 2.x forwardZeroPowerAcceleration(-57.3022), sign flipped
    c.naturalStrafeDeceleration.set(64.598);        // 2.x lateralZeroPowerAcceleration(-64.598), sign flipped

    // 2.x PIDF gains as stateless bring-up controllers (Foresight evaluates heading twice per update, so a stateful
    // Controller.pid(...) D term would differentiate across two different errors; AutoTune emits proportional ones):
    c.headingFeedback.set(Controller.proportional(1.6));   // 2.x headingPIDF P=1.6 (D=0.07 dropped, F=0)
    c.forwardTranslational.set(                             // 2.x translationalPIDF P=0.14, F=0.02 (D=0.02 dropped)
        Controller.proportional(0.14).plus(Controller.staticFeedforward(0.02)));
    c.strafeTranslational.set(
        Controller.proportional(0.14).plus(Controller.staticFeedforward(0.02)));

    // ESTIMATE (no 2.x equivalent; 2.x drivePIDF and mass have none): velocity -> power maps and brake model.
    c.coast.set(Controller.proportionalFeedforward(1.0 / 80.21));   // ~1/maxForwardVelocity
    c.brake.set(Controller.proportionalFeedforward(1.0 / 80.21));
    // Stopping distance d(v) = linear*v + quadratic*v|v| per body axis. Seeded with the 2.x zero-power (coast) model
    // d = v^2 / (2 * natural decel): over-predicts the braked distance, so the robot starts braking early (safe).
    c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
    c.quadraticBrakeCoefficients.set(Matrix.diag(1.0 / (2 * 57.3022), 1.0 / (2 * 64.598)));
    c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));   // no heading-stop prediction until tuned

    // 2.x PathConstraints(0.99, 100, 1, 1): tValue 0.99 -> 1 - 0.99; velocity 0.1 / translational 0.1 /
    // heading 0.007 / timeout 100 are the 3.x defaults already; brakingStrength/brakingStart have no equivalent
    // (closest: brakeAggression, default 1.0 = no bias).
    c.parametricTConstraint.set(0.01);
  });

  public static Localizer createLocalizer(HardwareMap hardwareMap) {
    return new PinpointLocalizer(hardwareMap, localizerConfig);
  }

  public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
    return new Mecanum(hardwareMap, drivetrainConfig);
  }

  /** Same name as in 2.x so Robot.java and the simulator keep calling it. Argument order: localizer, drivetrain, algorithm. */
  public static Follower createFollower(HardwareMap hardwareMap) {
    return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), new Foresight(foresightConfig));
  }
}
