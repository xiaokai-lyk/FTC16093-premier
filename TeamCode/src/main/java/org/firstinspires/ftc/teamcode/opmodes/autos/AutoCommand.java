package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;


public class AutoCommand {
    FrontArm frontArm;
    LiftArm liftArm;
    Follower follower;

    public AutoCommand(FrontArm frontArm, LiftArm liftArm) {
        this.frontArm = frontArm;
        this.liftArm = liftArm;
//        this.frontArmState = FrontArm.State.FREE;
    }

    /*--------------SAMPLE----------------*/
    public Command autoReleasePreloadSample(){
        return new SequentialCommandGroup(
                new WaitCommand(80),
                liftArm.releaseHigh(),
                new WaitCommand(150),
                liftArm.releaseHigh()
//                new WaitCommand(80)
        );
    }

    public Command autoIntakeSample() {
        return new SequentialCommandGroup(
                new WaitCommand(1100),
                frontArm.intake(true,true),
                new WaitCommand(350),
                frontArm.intake(true, true),
                new WaitCommand(50),
                new ParallelCommandGroup(frontArm.handover(),liftArm.handover())
        );
    }

    public Command autoReleaseHigh() {
        return liftArm.releaseHigh().andThen(
                new WaitCommand(180),
                liftArm.releaseHigh()
//                new WaitCommand(0)
        );
    }

    public Command autoIntakeLastSample(){
        return new SequentialCommandGroup(
                new WaitCommand(680),
                frontArm.intake(true,true),
                new WaitCommand(350),
                frontArm.intake(true, true),
                new WaitCommand(50),
                new ParallelCommandGroup(frontArm.handover(),liftArm.handover())
        );
    }


    /*--------------SPECIMEN----------------*/
    public Command scorePreloadSpecimen(){
        return new SequentialCommandGroup(
                frontArm.highChamber(),
                liftArm.highChamber(),
                new WaitCommand(120),
                liftArm.highChamber(),
                new WaitCommand(100)
        );
    }

    public Command autoIntakeSampleForHP(){
        return new SequentialCommandGroup(
                new WaitCommand(1200),
                frontArm.intake(true,true),
                new WaitCommand(280),
                frontArm.intake(true, true),
                new WaitCommand(50)
        );
    }

    public Command putSampleToHPCommand(){
        return new SequentialCommandGroup(
                new WaitCommand(500),
                frontArm.giveHP(),
                new WaitCommand(200)
        );
    }

    public Command autoIntakeSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(800),
                liftArm.highChamber(),
                new WaitCommand(500)
        );
    }

    public Command autoScoreSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(1000),
                frontArm.highChamber(),
                new WaitCommand(500),
                liftArm.highChamber(),
                new WaitCommand(1000)
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
