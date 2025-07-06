package org.firstinspires.ftc.teamcode.opmodes.autos;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.SpinnerConstant;
import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;


public class AutoCommand {
    FrontArm frontArm;
    LiftArm.LiftArmState liftArmState;
    FrontArm.State frontArmState;
    LiftArm liftArm;
    boolean flagRelease = true;

    public AutoCommand(FrontArm frontArm, LiftArm liftArm) {
        this.frontArm = frontArm;
        this.liftArm = liftArm;
//        this.frontArmState = FrontArm.State.FREE;
    }

    public Command autoIntakeSample() {
        return new SequentialCommandGroup(
                frontArm.intake(true, true),
                frontArm.intake(true, true),
                new ParallelCommandGroup(frontArm.handover(),liftArm.handover())
        );
//        return frontArm.intake(true, true).andThen(new ParallelCommandGroup(frontArm.handover(),liftArm.handover()));
        /*return new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> frontArm.open_claw(true)),
                        new InstantCommand(() -> {
                            frontArm.set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                            frontArm.set_arm_wrist(ServoConstants.ARM_WRIST_PREINTAKE);
                            frontArm.set_wrist(ServoConstants.WRIST_DOWN);
                            frontArm.set_spinner(SpinnerConstant.PARALLEL);
                        })
                ),
                new InstantCommand(() -> frontArm.getFrontSlide().setTargetPosition(MotorConstants.FRONT_MAX.value)),
//                new WaitUntilCommand(() ->
//                        (frontArm.getFrontSlide().getCurrentPosition() > abs(0.9 * MotorConstants.FRONT_MAX.value - MotorConstants.FRONT_TOLERANCE.value))
//                ),
                new WaitCommand(1000),
                new InstantCommand(()->{
                    frontArm.getArmWrist().setPosition(ServoConstants.ARM_WRIST_DOWN.value);
                    frontArm.set_wrist(ServoConstants.WRIST_DOWN);
                    new WaitCommand(100);
                    frontArm.open_claw(false);
                }).andThen(
                        new InstantCommand(()->{
                            frontArm.set_wrist(ServoConstants.WRIST_HANDOVER);
                            frontArm.set_spinner(SpinnerConstant.PARALLEL);
                            frontArm.set_arm_wrist(ServoConstants.ARM_WRIST_HANDOVER);
                            frontArm.set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                            frontArm.getFrontSlide().setTargetPosition(0);
                        }),
                        new WaitUntilCommand(()-> Math.abs(frontArm.getFrontSlide().getCurrentPosition() - frontArm.getFrontSlide().getTargetPosition())<10)
                ).andThen(
                        new SequentialCommandGroup(
                                new InstantCommand(()->liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_CLOSE.value)),
                                new WaitCommand(100),
                                new InstantCommand(()->frontArm.open_claw(true))
                        )
                )
        );*/
    }

//    public Command autoHandover(){
//        return new SequentialCommandGroup(
//                new WaitCommand(400),
//                new InstantCommand(()->liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_CLOSE.value)),
//                new WaitCommand(200),
//                new InstantCommand(()->frontArm.open_claw(true))
//        );
//    }

    public Command autoReleaseHigh() {
        return new SequentialCommandGroup(liftArm.releaseHigh(), new WaitCommand(2000), liftArm.releaseHigh());
//        return new ConditionalCommand(
//                liftArm.reachHighBasket()
//                        .alongWith(
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(()->liftArm.isFinished(MotorConstants.LIFT_ABOVE_BASKET_TOLERANCE.value)),
//                                        new InstantCommand(()->{
//                                            liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_BASKET.value);
//                                            liftArm.getWristUp().setPosition(ServoConstants.UP_WRIST_BASKET.value);
//                                        }),
//                                        new WaitCommand(100),
//                                        new InstantCommand(()->liftArm.getSlideUp().setPosition(ServoConstants.UP_SLIDE_MAX.value))
//                                ),
//                                new WaitUntilCommand(liftArm::isFinished)
//                        ),
//                new SequentialCommandGroup(
//                        new InstantCommand(()->liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_OPEN.value)),
//                        new WaitCommand(100),
//                        new InstantCommand(()->liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_PARALLEL.value)),
//                        new WaitCommand(150),
//                        new InstantCommand(()->liftArm.getSlideUp().setPosition(ServoConstants.UP_SLIDE_MIN.value)),
//                        new WaitCommand(100),
//                        new InstantCommand(()->{
//                            liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_OPEN.value);
//                            liftArm.getArmUp().setPosition(ServoConstants.UP_ARM_PARALLEL.value);
//                            liftArm.getWristUp().setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
//                        }),
//                        new InstantCommand(liftArm::resetSlide),
//                        new WaitUntilCommand(liftArm::isFinished),
//                        new InstantCommand(()->this.liftArmState = LiftArm.LiftArmState.FREE)
//                ),
//                ()->this.liftArm.getPosition()<0.95*MotorConstants.LIFT_HIGH.value
//        );
    }

    public Command autoIntakeSpecimen(){
        return new SequentialCommandGroup(
                liftArm.highChamber(),
                liftArm.highChamber()
        );
    }

    public Command autoScoreSpecimen(){
        return liftArm.highChamber();
    }

    public Command autoDriveCommmand(PathChain pathChain, Follower follower){
        return new InstantCommand(()->follower.followPath(pathChain));
    }
}
