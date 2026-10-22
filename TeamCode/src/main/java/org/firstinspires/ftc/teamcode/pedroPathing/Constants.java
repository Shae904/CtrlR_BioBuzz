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
  // spin 180 deg  CCW, then paste its block here.
  public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);   // 32 mm wheel, 2000 CPR
    c.xPodOffset.set(7.48184);
    c.yPodOffset.set(2.46908);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
  });

  // ---- Algorithm (2.x FollowerConstants + PathConstraints) --------------------------------------------------------
  public static ForesightConfig foresightConfig = new ForesightConfig(
          c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.4558351910046569);
            Controller secondaryTranslationalForward = Controller.proportional(0.1684189229355922);
            Controller primaryTranslationalLateral = Controller.proportional(0.691455510377392);
            Controller secondaryTranslationalLateral = Controller.proportional(0.2554743350529311);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.014309236183918803));
            c.brake.set(Controller.proportionalFeedforward(0.012162850756330983));

            c.headingFeedback.set(Controller.proportional(6.383694091682272));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04459341175390769, 0.005695790625929388));

            c.linearBrakeCoefficients.set(Matrix.diag(0.11232686228669889, 0.04853571666176151));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.00119311498172834, 0.0019764676218240436));

            c.maxAchievableForwardVelocity.set(72.71106855519396);
            c.maxAchievableStrafeVelocity.set(56.63592285408346);
            c.naturalForwardDeceleration.set(33.597201318488835);
            c.naturalStrafeDeceleration.set(77.99747854763653);
          }
  );

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
