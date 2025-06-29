package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(group = "0-competition", name = "16093 Solo")
public class TeleOpBase extends CommandOpModeEx {
    GamepadEx gamepadEx1;
    NewMecanumDrive driveCore;
    FrontArm frontArm;
    LiftArm liftArm;
    boolean isSample;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);


        driveCore = new NewMecanumDrive(hardwareMap);
        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(driveCore,
                ()->gamepadEx1.getLeftX(),
                ()->gamepadEx1.getLeftY(),
                ()->frontArm.state== FrontArm.State.DOWN?(gamepadEx1.getRightX()*0.45):gamepadEx1.getRightX(),
                ()->(gamepadEx1.getButton(GamepadKeys.Button.START)),
                ()->frontArm.state==FrontArm.State.DOWN?0.3:1);

        frontArm = new FrontArm(hardwareMap);
        liftArm = new LiftArm(hardwareMap);

        isSample = false;


        driveCore.resetHeading();
        driveCore.resetOdo();
        driveCore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CommandScheduler.getInstance().schedule(driveCommand);

        //timers
        new ButtonEx(()->getRuntime()>30).whenPressed(()->gamepad1.rumble(500));
        new ButtonEx(()->getRuntime()>60).whenPressed(()->gamepad1.rumble(500));
        new ButtonEx(()->getRuntime()>110).whenPressed(()->gamepad1.rumble(1000));


        new ButtonEx(()->gamepad1.touchpad).whenPressed(()-> {
            isSample = !isSample;
            gamepad1.rumble(100);
        });
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.BACK))
                .whenPressed(()->{
                    liftArm.initPos();
                    frontArm.initPos();
                });
    }

    @Override
    public void onStart() {
        liftArm.initPos();
        frontArm.initPos();
        resetRuntime();
    }

    @Override
    public void functionalButtons() {
        //Sample
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER) && frontArm.state != FrontArm.State.DOWN && isSample)
                .whenPressed(new ParallelCommandGroup(liftArm.releaseHigh(), new InstantCommand(frontArm::initPos)));

        //Specimen
        new ButtonEx(()->(gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && frontArm.state == FrontArm.State.HOLDING_BLOCK) && !isSample)
                .whenPressed(frontArm.giveHP());
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && frontArm.state != FrontArm.State.DOWN && !isSample)
                .whenPressed(liftArm.highChamber());


        //Shared
        new ButtonEx(()->(gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && frontArm.state == FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(true));
        new ButtonEx(()->(gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && frontArm.state == FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(false));

        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5)).whenPressed(
                frontArm.intake(true).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state == FrontArm.State.HOLDING_BLOCK && isSample))
        );
        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5)).whenPressed(
                frontArm.intake(false).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state==FrontArm.State.HOLDING_BLOCK && isSample))
        );
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        telemetry.addData("claw open", frontArm.claw_open);
        telemetry.addData("front arm state", frontArm.state);
        telemetry.addData("lift arm state", liftArm.state);
        telemetry.addData("spinner pos", frontArm.CurrentSpinnerPos);
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("claw deg", frontArm.getClawDeg());
        telemetry.addData("mode", isSample?"sample":"specimen");
        telemetry.update();
    }
}