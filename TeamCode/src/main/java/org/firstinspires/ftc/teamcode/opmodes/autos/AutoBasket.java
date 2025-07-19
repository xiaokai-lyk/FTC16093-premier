package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.localization.localizers.PinpointLocalizer;
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

@Autonomous(name = "Auto Basket", group = "Auto")
public class AutoBasket extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private FrontArm frontArm;
    private LiftArm liftArm;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(0, 114, Math.toRadians(-45));

    private final Pose scorePose = new Pose(2.2, 126, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(6.5, 117, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(7, 128, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(10.5, 124, Math.toRadians(29));
    private final Pose parkControlPose = new Pose(40, 126,Math.toRadians(-90));
    private final Pose parkPose = new Pose(65, 65, Math.toRadians(-90));
    private int currentPathId = 0;


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);
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

        frontArm.initPos();
        liftArm.autoInitPos();

        follower.setMaxPower(0.9);
    }

    @NonNull
    private Point getCurrentPoint(){
        return new Point(follower.getPose().getX(),follower.getPose().getY());
    }

    private double getCurrentHeading(){
        return follower.getPose().getHeading();
    }

    private void buildPaths() {
        PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;

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
                .addPath(new BezierLine(getCurrentPoint(), new Point(pickup2Pose)))
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

        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(getCurrentPoint(), new Point(parkControlPose), new Point(parkPose))))
                .setLinearHeadingInterpolation(getCurrentHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3);
    }

    @NonNull
    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeSampleCommand, releasePreloadCommand, releaseCommand, parkCommand;
        intakeSampleCommand = autoCommand.autoIntakeSample().andThen(actionEnd());
        releaseCommand = autoCommand.autoReleaseHigh().andThen(actionEnd());
        releasePreloadCommand = autoCommand.autoReleasePreloadSample().andThen(actionEnd());
        parkCommand = liftArm.parkCommand().andThen(actionEnd());

        actions.addAll(Arrays.asList(releaseCommand,
                intakeSampleCommand, releaseCommand,
                intakeSampleCommand, releaseCommand,
                intakeSampleCommand, releaseCommand));
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
        while (it.hasNext()){
            if (!opModeIsActive())break;
            periodic();
            if(follower.isFinished && !this.actionRunning){
                PathChain path = it.next();
//                if(path!=null) follower.follow(path, 1, 1, Math.toRadians(5), 1);
                if(path!=null) follower.followPath(path);
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
