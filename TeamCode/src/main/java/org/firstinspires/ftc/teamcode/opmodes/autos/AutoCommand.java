package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;

import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;


public class AutoCommand {
    FrontArm frontArm;
    LiftArm liftArm;
    Follower follower;
    boolean visionSucceed = false;
    int failedTime = 0;
    public AutoCommand(FrontArm frontArm, LiftArm liftArm) {
        this.frontArm = frontArm;
        this.liftArm = liftArm;
//        this.frontArmState = FrontArm.State.FREE;
    }

    /*--------------SAMPLE----------------*/
    public Command autoReleasePreloadSample_1(){
        return new SequentialCommandGroup(
//                new WaitCommand(100),
                liftArm.releaseHigh()
        );
    }
    public Command autoReleasePreloadSample_2(){
        return new SequentialCommandGroup(
                new WaitCommand(120),
                liftArm.releaseHigh()
        );
    }
    public Command autoIntakeSample() {
        return new SequentialCommandGroup(
                new WaitCommand(2000),
                frontArm.intake(true,true),
                new WaitCommand(180),
                frontArm.intake(true, true),
                new WaitCommand(50),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(frontArm.handover_1(),liftArm.handover_1()),
                        new SequentialCommandGroup(liftArm.handover_2(), frontArm.handover_2())
                )
        );
    }

    public Command autoReleaseHigh() {
        return new WaitCommand(300)
                .andThen(liftArm.releaseHigh())
                .andThen(
                new WaitCommand(350),
                liftArm.releaseHigh()
//                new WaitCommand(0)
        );
    }

    public Command autoIntakeLastSample(){
        return new SequentialCommandGroup(
                new WaitCommand(800),
                frontArm.intake(true,true),
                new WaitCommand(180),
                frontArm.intake(true, true),
                new WaitCommand(50),
                new ParallelCommandGroup(frontArm.handover(),liftArm.handover())
        );
    }


    /*--------------SPECIMEN----------------*/
    public Command scorePreloadSpecimen(){
        return new SequentialCommandGroup(
                liftArm.FirstHighChamber(),
                new WaitCommand(400),
                liftArm.highChamber()
        );
    }

    public Command autoIntakeSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(100),
                liftArm.highChamber()
        );
    }

    public Command autoScoreSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(80),
                liftArm.highChamber()
        );
    }

    public Command autoScoreLastSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(80),
                new InstantCommand(()->liftArm.getClawUp().setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                liftArm.resetSlideForAutoChamberEnd()
        );
    }



    /*--------------SAMPLE VERSION2----------------*/
    public Command autoReleasePreloadSample_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(80),
                liftArm.releaseHigh(),
                new WaitCommand(150),
                liftArm.releaseHigh()
//                new WaitCommand(80)
        );
    }

    public Command autoIntakeSample_v2() {
        return new SequentialCommandGroup(
                new WaitCommand(50),
                frontArm.intake(true,true),
                new WaitCommand(350),
                frontArm.intake(true, true),
                new WaitCommand(50),
                new ParallelCommandGroup(frontArm.handover(),liftArm.handover())
        );
    }

    public Command autoReleaseHigh_v2() {
        return liftArm.releaseHigh().andThen(
                new WaitCommand(180),
                liftArm.releaseHigh()
//                new WaitCommand(0)
        );
    }

    public Command autoIntakeLastSample_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(80),
                frontArm.intake(true,true),
                new WaitCommand(350),
                frontArm.intake(true, true),
                new WaitCommand(50),
                new ParallelCommandGroup(frontArm.handover(),liftArm.handover())
        );
    }


    /*--------------SPECIMEN INTAKE VERSION----------------*/
    public Command scorePreloadSpecimen_v2(){
        return new SequentialCommandGroup(
                frontArm.highChamber(),
                liftArm.highChamber(),
                new WaitCommand(200),
                liftArm.highChamber()
        );
    }

    public Command autoIntakeFirstSampleForHP_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                frontArm.intake(true,true),
                new WaitCommand(100),
                frontArm.intake(true, true),
                new WaitCommand(50)
        );
    }

    public Command autoIntakeSampleForHP_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(600),
                frontArm.intake(true,true),
                new WaitCommand(100),
                frontArm.intake(true, true),
                new WaitCommand(50)
        );
    }

    public Command putSampleToHPCommand_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(300),
                frontArm.giveHP(),
                new WaitCommand(100)
        );
    }

    public Command autoIntakeFirstSpecimen_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(50),
                liftArm.highChamber(),
                new WaitCommand(50)
        );
    }

    public Command autoIntakeSpecimen_v2(){
        return new SequentialCommandGroup(
                new WaitCommand(700),
                liftArm.highChamber(),
                new WaitCommand(50)
        );
    }

    public Command autoScoreSpecimen_v2(){
        return new SequentialCommandGroup(
//                new WaitCommand(80),
                frontArm.highChamber(),
                new WaitCommand(180),
                liftArm.highChamber()
        );
    }


    /*--------------OTHERS----------------*/
    public Point midPoint(Pose start, Pose end){
        return new Point((start.getX()+end.getX())/2,
                (start.getY()+end.getY())/2);
    }

    public Pose2d poseToPose2d(@NonNull Pose pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    }

    public Pose pose2dToPose(@NonNull Pose2d pose2d) {
        return new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @NonNull
    public Point getCurrentPoint(){
        return new Point(follower.getPose().getX(),follower.getPose().getY());
    }
}
