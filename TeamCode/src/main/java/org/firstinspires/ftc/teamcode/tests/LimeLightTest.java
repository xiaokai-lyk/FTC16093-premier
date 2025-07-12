package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;

public class LimeLightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            vision.update();
        }
    }
}
