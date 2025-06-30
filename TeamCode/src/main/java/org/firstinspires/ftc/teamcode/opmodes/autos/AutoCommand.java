package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.SpinnerConstant;
import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.Subsystems.SuperStructure;

public class AutoCommand {
    FrontArm frontArm;
    FrontArm.State state;
    LiftArm liftArm;

    public Command autoIntake(boolean is_far) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> SuperStructure.getInstance().isFinished()),  //直到上升滑轨收回再进行下面的动作
                // 1. 如果不是DOWN，先准备
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    frontArm.set_wrist(ServoConstants.WRIST_DOWN);
                                    frontArm.set_arm_wrist(ServoConstants.ARM_WRIST_DOWN);
                                }),
                                new WaitCommand(30),
                                new InstantCommand(() -> ServoConstants.CLAW_CHECK.setToServo(SuperStructure.getInstance().claw)),
                                new WaitCommand(70),
                                new InstantCommand(() -> frontArm.open_claw(true)),
                                new InstantCommand(() -> {
                                    frontArm.set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                                    frontArm.set_arm_wrist(ServoConstants.ARM_WRIST_PREINTAKE);
                                    frontArm.set_wrist(ServoConstants.WRIST_DOWN);
                                    this.state = FrontArm.State.DOWN;
                                }),
                                new ConditionalCommand(
                                        new InstantCommand(() -> frontArm.set_spinner(SpinnerConstant.PARALLEL)),
                                        new InstantCommand(),
                                        () -> this.state != FrontArm.State.DOWN
                                ),
                                new InstantCommand(() -> this.state = FrontArm.State.DOWN)
                        ),
                        new InstantCommand(), // 如果已经是DOWN，什么都不做
                        () -> this.state != FrontArm.State.DOWN
                ),
                // 2. 滑轨位置调整
                new InstantCommand(() -> SuperStructure.getInstance().slideFront.setTargetPosition(is_far ? MotorConstants.FRONT_MAX.value : MotorConstants.FRONT_NEAR.value)),
                // 3. 等待滑轨到位
                new WaitUntilCommand(() ->
                        (is_far && (SuperStructure.getInstance().slideFront.getCurrentPosition() > 0.97 * MotorConstants.FRONT_MAX.value - MotorConstants.FRONT_TOLERANCE.value))
                                || (!is_far && SuperStructure.getInstance().slideFront.getCurrentPosition() < MotorConstants.FRONT_NEAR.value + 10)
                ),
                // 4. 执行抓取动作
                new InstantCommand(() -> frontArm.open_claw(false)),
                new WaitCommand(50),
                new InstantCommand(() -> this.state = FrontArm.State.HOLDING_BLOCK)
        );
    }

    public Command autoReleaseHigh() {
        return new ConditionalCommand(
            // 高位篮筐释放动作
            new SequentialCommandGroup(
                liftArm.highBasketCommand(),
                new WaitUntilCommand(() -> SuperStructure.getInstance().isFinished(MotorConstants.LIFT_ABOVE_BASKET_TOLERANCE.value)),
                new InstantCommand(() -> {
                    liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_BASKET.value);
                    liftArm.getWristUp().setPosition(ServoConstants.UP_WRIST_BASKET.value);
                }),
                new WaitCommand(100),
                new InstantCommand(() -> liftArm.getSlideUp().setPosition(ServoConstants.UP_SLIDE_MAX.value))
            ),
            // 回退动作
            new SequentialCommandGroup(
                new InstantCommand(() -> liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                new WaitCommand(100),
                new InstantCommand(() -> {
                    liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                    liftArm.getWristUp().setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                }),
                new WaitCommand(150),
                new InstantCommand(() -> liftArm.getSlideUp().setPosition(ServoConstants.UP_SLIDE_MIN.value)),
                new WaitCommand(100),
                new InstantCommand(() -> {
                    liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_OPEN.value);
                    liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                    liftArm.getWristUp().setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                }),
                new InstantCommand(() -> liftArm.resetSlide()),
                new WaitUntilCommand(() -> SuperStructure.getInstance().isFinished()),
                new InstantCommand(() -> liftArm.state = LiftArm.LiftArmState.FREE),
                // 尝试高位释放动作
                new SequentialCommandGroup(
                    liftArm.highBasketCommand(),
                    new WaitUntilCommand(() -> SuperStructure.getInstance().isFinished(MotorConstants.LIFT_ABOVE_BASKET_TOLERANCE.value)),
                    new InstantCommand(() -> {
                        liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_BASKET.value);
                        liftArm.getWristUp().setPosition(ServoConstants.UP_WRIST_BASKET.value);
                    }),
                    new WaitCommand(100),
                    new InstantCommand(() -> liftArm.getSlideUp().setPosition(ServoConstants.UP_SLIDE_MAX.value))
                )
            ),
            () -> liftArm.getPosition() >= 0.95 * MotorConstants.LIFT_HIGH.value
        );
    }
}
