package org.firstinspires.ftc.teamcode.opmodes.autos;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;


public class AutoCommand {
    FrontArm frontArm;
    LiftArm.LiftArmState liftArmState;
    FrontArm.State frontArmState;
    LiftArm liftArm;
    boolean flagRelease = true;
    Follower follower;

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
    }

    public Command autoIntakeSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(100),
                liftArm.highChamber(),
                new WaitCommand(100)
        );
    }

    public Command autoScoreSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(500),
                liftArm.highChamber(),
                new WaitCommand(100)
        );
    }

    public Command scorePreloadSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                liftArm.highChamber(),
                new WaitCommand(50)
        );
    }

    public Command autointakePreloadSpecimen(){
        return new SequentialCommandGroup(
                liftArm.highChamber(),
                new WaitCommand(800),
                liftArm.highChamber(),
                new WaitCommand(50)
        );
    }

    public Command autoIntakeSampleToHP(){
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                frontArm.intake(true, true),
                frontArm.intake(true, true),
                new InstantCommand(()->frontArm.getFrontSlide().setTargetPosition(0)),
                new WaitCommand(1000)
        );
    }

    public Command putSpecimenToHPCommand(){
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                frontArm.giveHP(),
                new WaitCommand(1000)
        );
    }

    public Command autoDriveCommmand(PathChain pathChain, Follower follower){
        return new InstantCommand(()->follower.followPath(pathChain));
    }

    public static Pose2d poseToPose2d(Pose pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static Pose pose2dToPose(Pose2d pose2d) {
        return new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public PathChain[] buildPathWithBrake(Pose startPose, Pose endPose) {
        Pose2d startPose2d = poseToPose2d(startPose);
        Pose2d endPose2d = poseToPose2d(endPose);

        Vector2d bufferVec = endPose2d.vec().minus(
                new Vector2d(2, 0).rotated(endPose.getHeading())
        );
        Pose2d bufferPose2d = new Pose2d(bufferVec, endPose.getHeading());
        Pose bufferPose = pose2dToPose(bufferPose2d);

        follower.setMaxPower(1);
        PathChain main = follower.pathBuilder()
                .addPath(new BezierLine(startPose, bufferPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        follower.setMaxPower(0.5);
        PathChain brake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bufferPose), new Point(endPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), endPose.getHeading())
                .build();

        return new PathChain[]{main, brake};
    }


    public Point midPoint(Pose start, Pose end){
        return new Point((start.getX()+end.getX())/2,
                (start.getY()+end.getY())/2);
    }
}
