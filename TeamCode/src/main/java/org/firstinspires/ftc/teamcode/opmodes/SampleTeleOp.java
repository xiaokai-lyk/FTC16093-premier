package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;

public class SampleTeleOp extends TeleOpBase{

    @Override
    public void addButtons() {
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.Y)).whenPressed(
                frontArm.intake(true).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state== FrontArm.State.HANDOVER))
        );
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.X)).whenPressed(
                frontArm.intake(false).andThen(new ConditionalCommand(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()),
                        new InstantCommand(),
                        ()->frontArm.state==FrontArm.State.HANDOVER))
        );
        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5 && frontArm.state== FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(true));
        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5 && frontArm.state== FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(false));
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)).whenPressed(new ParallelCommandGroup(liftArm.releaseHigh(), new InstantCommand(frontArm::initPos)));
    }
}
