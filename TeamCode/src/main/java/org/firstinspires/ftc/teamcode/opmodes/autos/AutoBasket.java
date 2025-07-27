package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.utils.FollowerEx;
import org.firstinspires.ftc.teamcode.utils.PathChainList;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Basket", group = "Competition")
public class AutoBasket extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private FrontArm frontArm;
    private LiftArm liftArm;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(0, 114, Math.toRadians(-45));

    private final Pose scorePose = new Pose(2.7, 126, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(6, 117.5, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(6, 126.6, Math.toRadians(0.5));
    private final Pose pickup3Pose = new Pose(9.5, 124, Math.toRadians(29.3));
    private final Pose parkControlPose = new Pose(46, 126,Math.toRadians(-90));
    private final Pose parkPose = new Pose(50, 85, Math.toRadians(-90));
    private int currentPathId = 0;


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new FollowerEx(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        frontArm = new FrontArm(hardwareMap);
        liftArm = new LiftArm(hardwareMap);
        this.pathChainList = new PathChainList();
        this.actions = new ArrayList<>();
        this.autoCommand = new AutoCommand(frontArm, liftArm);
        this.actionRunning = false;

        buildPaths();
        buildActions();

        frontArm.setLED(false);
        frontArm.autoInitPos();
        liftArm.autoInitPos();

        follower.setMaxPower(1);
    }

    @NonNull
    private Point getCurrentPoint(){
        return new Point(follower.getPose().getX(),follower.getPose().getY());
    }

    private double getCurrentHeading(){
        return follower.getPose().getHeading();
    }

    private void buildPaths() {
        PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park1, park2;

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(scorePose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(getCurrentPoint(), autoCommand.midPoint(scorePose, pickup2Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(scorePose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPoint(), new Point(scorePose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), scorePose.getHeading())
                .build();

        park1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(scorePose), new Point(parkControlPose))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkControlPose.getHeading())
                .build();

        park2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(parkControlPose), new Point(parkPose))))
                .setLinearHeadingInterpolation(parkControlPose.getHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(null, scorePreload, null,
                grabPickup1, scorePickup1,
                grabPickup2, scorePickup2,
                grabPickup3, scorePickup3,
                park1, park2, null);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeSampleCommand, releasePreloadCommand_1, releasePreloadCommand_2, releaseCommand, parkCommand, intakeLastSampleCommand;
        intakeSampleCommand = autoCommand.autoIntakeSample().andThen(actionEnd());
        releaseCommand = autoCommand.autoReleaseHigh().andThen(actionEnd());
        releasePreloadCommand_1 = autoCommand.autoReleasePreloadSample_1().andThen(actionEnd());
        releasePreloadCommand_2 = autoCommand.autoReleasePreloadSample_2().andThen(actionEnd());

        parkCommand = liftArm.parkCommand().andThen(actionEnd());
        intakeLastSampleCommand = autoCommand.autoIntakeLastSample().andThen(actionEnd());

        actions.addAll(Arrays.asList(releasePreloadCommand_1, null, releasePreloadCommand_2,
                intakeSampleCommand, releaseCommand,
                intakeSampleCommand, releaseCommand,
                intakeSampleCommand, releaseCommand,
                null, parkCommand, new WaitCommand(999999)));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("drive error",follower.driveError);
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("follower finished",follower.isFinished);
        telemetry.addData("action finished", !this.actionRunning);
        telemetry.addData("current path /id", currentPathId);
        telemetry.addData("front arm", frontArm.state);
        telemetry.addData("lift arm", liftArm.state);
        telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());
        telemetry.update();
    }

    @Override
    public void run() {
        if(actions.size() != pathChainList.size()){
            throw new IllegalStateException(
                    "Actions count (" + actions.size() +
                            ") does not match path count (" + pathChainList.size() + ")"
            );
        }
        Iterator<PathChain> it = pathChainList.iterator();
        int pathCount = 0;
        while (it.hasNext()){
            pathCount+=1;
            if (!opModeIsActive())break;
            periodic();
            if(!follower.isBusy() && !this.actionRunning){
                PathChain path = it.next();
//                if(path!=null) follower.follow(path, 1, 0.5, Math.toRadians(0), 1);
                if(path!=null){
                    if (pathCount==11){
                        follower.followPath(path,0.1,false);
                    }
                    else follower.followPath(path, 1,true);
                }
                Command currentAction = actions.get(currentPathId);
                if(currentAction!=null){
                    currentAction.schedule();
                    this.actionRunning = true;
                }
                currentPathId++;
            }
        }
    }
}