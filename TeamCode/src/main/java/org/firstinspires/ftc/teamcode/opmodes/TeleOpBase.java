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

@Config
@TeleOp(name = "16093 TeleOp", group = "Competition")
public class TeleOpBase extends CommandOpModeEx {
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    private NewMecanumDrive driveCore;
    private FrontArm frontArm;
    private LiftArm liftArm;
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
                ()->gamepadEx1.getRightX(),
                ()->(gamepadEx1.getButton(Button.START) && !gamepad1.touchpad),
                ()->1-gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        frontArm = new FrontArm(hardwareMap);
        liftArm = new LiftArm(hardwareMap);


        driveCore.resetHeading();
        driveCore.resetOdo();
        driveCore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CommandScheduler.getInstance().schedule(driveCommand);

        /// buttons ///
        new ButtonEx(()->gamepadEx1.getButton(Button.Y)).whenPressed(
                frontArm.intake(true).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state==FrontArm.State.HANDOVER))
        );
        new ButtonEx(()->gamepadEx1.getButton(Button.X)).whenPressed(
                frontArm.intake(false).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state==FrontArm.State.HANDOVER))
        );
        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5 && frontArm.state== FrontArm.State.DOWN))
                .whenPressed(frontArm::spinner_rotate);
        new ButtonEx(()->(gamepadEx1.getButton(Button.DPAD_RIGHT) && frontArm.state== FrontArm.State.FREE)).whenPressed(
                frontArm.giveHP()
        );
        new ButtonEx(()->gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5).whenPressed(new ParallelCommandGroup(liftArm.releaseHigh(), new InstantCommand(frontArm::initPos)));
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
        telemetry.update();
    }
}