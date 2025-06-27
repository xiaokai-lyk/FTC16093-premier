package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;

public abstract class TeleOpBase extends CommandOpModeEx {
    GamepadEx gamepadEx2;
    GamepadEx gamepadEx1;
    NewMecanumDrive driveCore;
    FrontArm frontArm;
    LiftArm liftArm;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


        driveCore = new NewMecanumDrive(hardwareMap);
        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(driveCore,
                ()->gamepadEx1.getLeftX(),
                ()->gamepadEx1.getLeftY(),
                ()->frontArm.state== FrontArm.State.DOWN?(gamepadEx1.getRightX()*0.45):gamepadEx1.getRightX(),
                ()->(gamepadEx1.getButton(GamepadKeys.Button.START) && !gamepad1.touchpad),
                ()->frontArm.state==FrontArm.State.DOWN?0.3:1);

        frontArm = new FrontArm(hardwareMap);
        liftArm = new LiftArm(hardwareMap);


        driveCore.resetHeading();
        driveCore.resetOdo();
        driveCore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void onStart() {
        liftArm.initPos();
        frontArm.initPos();
    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        telemetry.addData("claw open", frontArm.claw_open);
        telemetry.addData("front arm state", frontArm.state);
        telemetry.addData("spinner pos", frontArm.CurrentSpinnerPos);
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("claw deg", frontArm.getClawDeg());
        telemetry.update();
    }
}