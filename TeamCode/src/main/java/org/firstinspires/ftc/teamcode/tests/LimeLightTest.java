package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@TeleOp(group = "tests", name = "Lime Light Test")
public class LimeLightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, telemetry);
        vision.initialize();
        vision.setColor(2);
        waitForStart();
        while (opModeIsActive()){
            vision.update();
            telemetry.update();
        }
    }
}
