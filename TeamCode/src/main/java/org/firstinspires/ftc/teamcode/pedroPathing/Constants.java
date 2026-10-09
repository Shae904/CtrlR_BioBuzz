package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants()
      .mass(12.345)
      .forwardZeroPowerAcceleration(-57.3022).lateralZeroPowerAcceleration(-64.598)
      .headingPIDFCoefficients(new PIDFCoefficients(1.6, 0, .07, 0))
      .translationalPIDFCoefficients(new PIDFCoefficients(.14, 0, 0.02, 0.02))
      .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.018, 0, 0.0009, .6, .0)); // .0072

  public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .rightFrontMotorName("fr")
      .rightRearMotorName("br")
      .leftRearMotorName("bl")
      .leftFrontMotorName("fl")
      .xVelocity(80.21)
      .yVelocity(65.9686)
      .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

  public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
      .forwardTicksToInches(.001989436789)
      .strafeTicksToInches(.001989436789)
      .turnTicksToInches(.001989436789)
      .leftPodY(7.5)
      .rightPodY(-7.5)
      .strafePodX(-2)
      .leftEncoder_HardwareMapName("fl")
      .rightEncoder_HardwareMapName("fr")
      .strafeEncoder_HardwareMapName("flywheel2")
      .leftEncoderDirection(Encoder.REVERSE)
      .rightEncoderDirection(Encoder.REVERSE)
      .strafeEncoderDirection(Encoder.REVERSE);

  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pathConstraints(pathConstraints)
        .threeWheelLocalizer(localizerConstants)
        .mecanumDrivetrain(driveConstants)
        .build();
  }
}
