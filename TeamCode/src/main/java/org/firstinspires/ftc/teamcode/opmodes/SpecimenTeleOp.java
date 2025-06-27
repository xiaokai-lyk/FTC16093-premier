package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.utils.ButtonEx;
@TeleOp(group = "0-competition", name = "Specimen")
public class SpecimenTeleOp extends TeleOpBase{

    @Override
    public void addButtons() {
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.Y))
                .whenPressed(frontArm.intake(true));
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.X))
                .whenPressed(frontArm.intake(false));
        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5 && frontArm.state== FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(true));
        new ButtonEx(()->(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5 && frontArm.state== FrontArm.State.DOWN))
                .whenPressed(()->frontArm.spinner_rotate(false));
        new ButtonEx(()->(gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER) && frontArm.state== FrontArm.State.FREE))
                .whenPressed(frontArm.giveHP());
        new ButtonEx(()->gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(liftArm.getFromWall());
    }
}
