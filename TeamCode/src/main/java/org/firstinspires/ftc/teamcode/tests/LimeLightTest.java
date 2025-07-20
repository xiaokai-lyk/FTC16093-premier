package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@TeleOp(group = "tests", name = "Lime Light Test")
public class LimeLightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, telemetry);
        waitForStart();
        vision.initialize();
        vision.setColorVal(2);
        while (opModeIsActive()){
            if(gamepad1.a)vision.setLed(true);
            if(gamepad1.b)vision.setLed(false);
            vision.update(true);
            telemetry.update();
        }
    }
}
