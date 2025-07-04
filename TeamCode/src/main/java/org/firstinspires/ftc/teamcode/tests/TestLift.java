package org.firstinspires.ftc.teamcode.tests;


import android.widget.Button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestLift", group="Test")
public class TestLift extends LinearOpMode {
    DcMotor liftMotor;

    @Override
    public void runOpMode() {
        // 初始化硬件
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 目标位置（根据实际情况调整）
        int targetPosition = 1000;
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setPower(1.0);

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        // 只 schedule 一次，不在 while 循环里反复 schedule
        // 只要 OpMode 没结束，电机会自己跑到目标
        while (opModeIsActive() && liftMotor.isBusy()) {
            telemetry.addData("Current Position", liftMotor.getCurrentPosition());
            telemetry.update();
        }

        liftMotor.setPower(0);
        telemetry.addLine("Done");
        telemetry.update();
    }
}