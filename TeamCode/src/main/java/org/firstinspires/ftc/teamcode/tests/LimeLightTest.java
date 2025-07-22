package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@TeleOp(group = "tests", name = "Lime Light Test")
public class LimeLightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, telemetry);
        Servo spinner = hardwareMap.get(Servo.class, "armSpin");
        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slideFront");
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setPower(0);
        waitForStart();
        vision.initialize();
        vision.setColorVal(2);
        while (opModeIsActive()){
            if(gamepad1.a)vision.setLed(true);
            if(gamepad1.b)vision.setLed(false);
            if(gamepad1.touchpad){
                LLResult res = vision.getResult();
                if(vision.resultValid(res))spinner.setPosition(vision.getArmSpinnerPos(res));
            }
            if(gamepad1.dpad_up){
                slide.setPower(1);
                LLResult res = vision.getResult();
                if(vision.resultValid(res))slide.setTargetPosition(vision.getSlideTarget(res));
            }
            if(gamepad1.dpad_down){
                slide.setPower(0);
            }
            vision.update(true);
            telemetry.addData("slide pos", slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
