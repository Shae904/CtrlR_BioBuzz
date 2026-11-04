package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import java.util.List;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Robot {

  private final List<LynxModule> hubs;
  public Follower follower;
  public DcMotor fr, fl, br, bl;
  public IMU imu;

  public ServoImplEx tiltA, tiltB;

  public static double TILT_A_FOLDED = .98;

  public static double TILT_A_DEPLOYED = 0.49;
  public static double TILT_B_FOLDED = .08;
  public static double TILT_B_DEPLOYED = 0.55;

  public Outtake outtake;
  public Intake intake;
  public Limelight limelight;


  private final AllianceColor allianceColor;

  public Robot(LinearOpMode opMode) {
    this(opMode, AllianceColor.RED);
  }

  public Robot(LinearOpMode opMode, AllianceColor allianceColor) {
    this.allianceColor = allianceColor;

    HardwareMap hardwareMap = opMode.hardwareMap;
    this.follower = Constants.createFollower(hardwareMap);

    // From https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
    this.hubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : this.hubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    // TODO: what is this
    opMode.telemetry.setMsTransmissionInterval(11);
    // Send this opmode's telemetry to Panels too: Panels' Graph only plots "name: number" lines that reach Panels
    // telemetry, and the plain Driver Station telemetry (Vel Current, Vel Target, ...) never does.
    opMode.telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), opMode.telemetry);

    bl = hardwareMap.dcMotor.get("bl");
    br = hardwareMap.dcMotor.get("br");
    fl = hardwareMap.dcMotor.get("fl");
    fr = hardwareMap.dcMotor.get("fr");

    fl.setDirection(Direction.REVERSE);
    fr.setDirection(Direction.FORWARD);
    bl.setDirection(Direction.REVERSE);
    br.setDirection(Direction.FORWARD);

    fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        LogoFacingDirection.BACKWARD,
        UsbFacingDirection.RIGHT));
    imu.initialize(parameters);
    opMode.telemetry.addData("IMU Initialized", true);
    opMode.telemetry.update();

    // Init Lift

    tiltA = hardwareMap.get(ServoImplEx.class, "tiltA");
    tiltB = hardwareMap.get(ServoImplEx.class, "tiltB");

    // Init Subsystems
    outtake = new Outtake(opMode);
    intake = new Intake(opMode);

    limelight = new Limelight(opMode, this.getAllianceColor());
  }

  public void initAuton() {
    this.intake.setFlapper(false);
    this.setTiltFolded();
  }

  public void setTiltFolded() {
    this.tiltA.setPosition(TILT_A_FOLDED);
    this.tiltB.setPosition(TILT_B_FOLDED);
  }

  public void setTiltDeployed() {
    this.tiltA.setPosition(TILT_A_DEPLOYED);
    this.tiltB.setPosition(TILT_B_DEPLOYED);
  }

  public void setTiltPos(double posA, double posB) {
    this.tiltA.setPosition(posA);
    this.tiltB.setPosition(posB);
  }

  public AllianceColor getAllianceColor() {
    return this.allianceColor;
  }

  public void setBulkCachingMode(LynxModule.BulkCachingMode mode) {
    for (LynxModule hub : this.hubs) {
      hub.setBulkCachingMode(mode);
    }
  }

  public void clearBulkCache() {
    for (LynxModule hub : this.hubs) {
      hub.clearBulkCache();
    }
  }

  /**
   * Turret: aims at the target CELL while {@code shouldAim} (Limelight: field-held aim, see TurretTracker), else back to
   * its rest direction. (The DECODE "reset" that faked the turret at 180 deg, gamepad1 square, spun it at full power
   * while held: removed.)
   */
  public void updateAimingSystem(boolean shouldAim) {
    double turretPosition = this.outtake.getTurretPosDegrees();
    if (shouldAim) {
      // Pedro's robot-frame velocity (in/s forward, in/s left, rad/s) and pose
      this.limelight.updateAim(this.follower.pose(), this.follower.twist(), turretPosition);
    }
    this.limelight.updateTarget(turretPosition, shouldAim, this.getAllianceColor());
    this.outtake.setPowerTurret(this.limelight.updateAimPID());
  }

  public void updateAutoControls() {
    this.follower.update();
    this.outtake.updatePIDControl();
    this.updateAimingSystem(true);

  }
}
