package org.firstinspires.ftc.teamcode.opmodes;

// Currently not working!!!
// Currently not working!!!
// Currently not working!!!
// Currently not working!!!
// Currently not working!!!
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;


@TeleOp(group = "0-competition", name = "16093 Dual")
public class TeleOpDual extends CommandOpModeEx {
    GamepadEx gamepadEx1, gamepadEx2;
    NewMecanumDrive driveCore;
    FrontArm frontArm;
    LiftArm liftArm;


    private enum Tasks{
        SAMPLE,
        SPECIMEN,
        ASCENT
    };
    private Tasks mode;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        this.mode = Tasks.SAMPLE;

        driveCore = new NewMecanumDrive(hardwareMap);
        driveCore.init();
        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(driveCore,
                ()->gamepadEx1.getLeftX(),
                ()->gamepadEx1.getLeftY(),
                ()->frontArm.state== FrontArm.State.DOWN?(gamepadEx1.getRightX()*0.45):gamepadEx1.getRightX(),
                ()->(gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)),
                ()->frontArm.state==FrontArm.State.DOWN?0.7:1);

        frontArm = new FrontArm(hardwareMap);
        liftArm = new LiftArm(hardwareMap);


        driveCore.resetHeading();
        driveCore.resetOdo();
        driveCore.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CommandScheduler.getInstance().schedule(driveCommand);



        new ButtonEx(()->gamepad1.touchpad).whenPressed(()-> {
            mode = (mode ==  Tasks.SAMPLE )? Tasks.SPECIMEN: Tasks.SAMPLE;
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
        new ButtonEx(()->gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5 && frontArm.state != FrontArm.State.DOWN&& mode == Tasks.SAMPLE)
                .whenPressed(new ParallelCommandGroup(liftArm.releaseHigh(), new InstantCommand(frontArm::initPos)));

        //Specimen
        new ButtonEx(()->(gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && frontArm.state == FrontArm.State.HOLDING_BLOCK)&& mode == Tasks.SPECIMEN)
                .whenPressed(frontArm.giveHP());
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && frontArm.state != FrontArm.State.DOWN&& mode == Tasks.SPECIMEN)
                .whenPressed(new SequentialCommandGroup(frontArm.highChamber(), liftArm.highChamber()));


        //Shared
        new ButtonEx(()->(gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && frontArm.state == FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(true));
        new ButtonEx(()->(gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && frontArm.state == FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(false));

        new ButtonEx(()->(gamepadEx2.getButton(GamepadKeys.Button.Y) && mode != Tasks.ASCENT && frontArm.state != FrontArm.State.DOWN)).whenPressed(
                frontArm.intake(true, false).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                                new InstantCommand(),
                                ()->frontArm.state == FrontArm.State.HOLDING_BLOCK&& mode == Tasks.SAMPLE))
                        .alongWith(
                                new ConditionalCommand(
                                        liftArm.highChamber(),
                                        new InstantCommand(),
                                        ()->liftArm.state== LiftArm.LiftArmState.PRE_CHAMBER
                                )
                        )
        );


        new ButtonEx(()->(gamepadEx2.getButton(GamepadKeys.Button.A) && mode != Tasks.ASCENT)).whenPressed(
                frontArm.intake(false, false).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state==FrontArm.State.HOLDING_BLOCK && mode == Tasks.SAMPLE))
                        .alongWith(
                                new ConditionalCommand(
                                        liftArm.highChamber(),
                                        new InstantCommand(),
                                        ()->liftArm.state== LiftArm.LiftArmState.PRE_CHAMBER
                                )
                        )
        );

        new ButtonEx(()->(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5 && mode != Tasks.ASCENT && frontArm.state == FrontArm.State.DOWN)).whenPressed(
                frontArm.intake(true, false).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                                new InstantCommand(),
                                ()->frontArm.state == FrontArm.State.HOLDING_BLOCK&& mode == Tasks.SAMPLE))
                        .alongWith(
                                new ConditionalCommand(
                                        liftArm.highChamber(),
                                        new InstantCommand(),
                                        ()->liftArm.state== LiftArm.LiftArmState.PRE_CHAMBER
                                )
                        )
        );

        //Ascent
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.B)).whenPressed(new SequentialCommandGroup(
                new InstantCommand(()->mode= Tasks.ASCENT),
                liftArm.ascent_up()
        ).andThen(
                new WaitUntilCommand(()->gamepadEx1.getButton(GamepadKeys.Button.B)),
                new InstantCommand(liftArm::hold_slide),
                new WaitUntilCommand(()->gamepadEx1.getButton(GamepadKeys.Button.B)),
                liftArm.ascent_down(),
                new WaitUntilCommand(()->gamepadEx1.getButton(GamepadKeys.Button.B)),
                new InstantCommand(liftArm::hold_slide).alongWith(liftArm.ascent_end())
        ));
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        telemetry.addData("mode", mode);
        telemetry.addData("front arm state", frontArm.state);
        telemetry.addData("lift arm state", liftArm.state);
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("claw deg", frontArm.getClawDeg());
        telemetry.update();
    }
}