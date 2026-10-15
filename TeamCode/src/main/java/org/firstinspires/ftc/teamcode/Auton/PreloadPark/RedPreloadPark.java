package org.firstinspires.ftc.teamcode.Auton.PreloadPark;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Auton.FarAuton.BaseFar15;
import org.firstinspires.ftc.teamcode.Robot;

@Configurable
@Autonomous(name = "RED PRELOAD PARK", group = "RED")
public class RedPreloadPark extends LinearOpMode {

    public static double[] SHOOT_RED = {52, 15, 110.5};

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, AllianceColor.RED);
        new BaseFar15(this, robot, SHOOT_RED).run();
    }
}