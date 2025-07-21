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

@Autonomous(name = "Auto Chamber pushSample", group = "Auto")
public class AutoChamber_pushSample extends AutoOpModeEx {
    private FollowerEx follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private FrontArm frontArm;
    private LiftArm liftArm;
    private Boolean actionRunning;


    /*
     * -----------------------
     * |                     |
     * |                     |
     * |      |-----|        |
     * |      |潜水器|        |
     * |      |-----|        |
     * |                     |
     * |         ↑           |
     * |       启动点         |
     * ---------------------零点
     * */





    private PathChainList pathChainList;


    private final Pose startPose = new Pose(0,  52.75, Math.toRadians(0));

    private final Pose midPoint = new Pose(21, 35, Math.toRadians(0));
    private final Pose push1Pose = new Pose(51, 24, Math.toRadians(0));
    private final Pose push2Pose = new Pose(51, 12, Math.toRadians(0));
    private final Pose push3Pose = new Pose(51, 6.5, Math.toRadians(0));

    private final Pose HPPose = new Pose(0, 29, Math.toRadians(0));
    private final Pose endPush1 = new Pose(12, 24, Math.toRadians(0));
    private final Pose endPush2 = new Pose(12, 12, Math.toRadians(0));
    private final Pose endPush3 = new Pose(12, 6.5, Math.toRadians(0));
//    private final Pose HPPoseForEndPush = new Pose(0,11,Math.toRadians(0));

    private final Pose scorePose0 = new Pose(27, 61, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(29, 64, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(29, 67, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(29, 70, Math.toRadians(0));

//    private final Pose parkControlPose = new Pose(, 25, Math.toRadians(0));
    private final Pose parkPose = new Pose(8, 28, Math.toRadians(0));
    private int currentPathId = 0;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap);
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

        frontArm.autoInitPos();
        liftArm.autoChamberInitPos();
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
        PathChain toMidPoint, toPush1, toPush2, toPush3,
                pushEnd1, pushEnd2, pushEnd3, goToHP,
                scoreChamber0, scoreChamber1, scoreChamber2, scoreChamber3, park;
        scoreChamber0 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose0)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose0.getHeading())
                .build();

        toMidPoint = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose0), autoCommand.midPoint(scorePose0, midPoint), new Point(midPoint)))
                .setLinearHeadingInterpolation(scorePose0.getHeading(), midPoint.getHeading())
                .build();

        toPush1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(midPoint), autoCommand.midPoint(midPoint, push1Pose), new Point(push1Pose)))
                .setLinearHeadingInterpolation(midPoint.getHeading(), push1Pose.getHeading())
                .build();

        pushEnd1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push1Pose),  new Point(endPush1)))
                .setLinearHeadingInterpolation(push1Pose.getHeading(), endPush1.getHeading())
                .build();

        toPush2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endPush1), autoCommand.midPoint(scorePose0, push2Pose), new Point(push2Pose)))
                .setLinearHeadingInterpolation(endPush1.getHeading(), push2Pose.getHeading())
                .build();

        pushEnd2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push2Pose),  new Point(endPush2)))
                .setLinearHeadingInterpolation(push2Pose.getHeading(), endPush2.getHeading())
                .build();

        toPush3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(endPush2), autoCommand.midPoint(endPush2, push3Pose), new Point(push3Pose)))
                .setLinearHeadingInterpolation(endPush2.getHeading(), push3Pose.getHeading())
                .build();

        pushEnd3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push3Pose),  new Point(endPush3)))
                .setLinearHeadingInterpolation(push3Pose.getHeading(), endPush3.getHeading())
                .build();

        goToHP = follower.pathBuilder()
                .addPath(new BezierCurve(getCurrentPoint(), autoCommand.midPoint(follower.getPose(), HPPose), new Point(HPPose)))
                .setLinearHeadingInterpolation(endPush3.getHeading(), HPPose.getHeading())
                .build();

        scoreChamber1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HPPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(HPPose.getHeading(), scorePose1.getHeading())
                .build();

        scoreChamber2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HPPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(HPPose.getHeading(), scorePose2.getHeading())
                .build();

        scoreChamber3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HPPose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(HPPose.getHeading(), scorePose3.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(scoreChamber0,
                toMidPoint, toPush1,
                pushEnd1, toPush2,
                pushEnd2, toPush3,
                pushEnd3, goToHP,
                null, scoreChamber1, null, goToHP,
                null, scoreChamber2, null, goToHP,
                null, scoreChamber3, null, park);
    }

    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeSpecimenCommand, scoreSpecimenCommand, scorePreloadCommand;
        scorePreloadCommand = autoCommand.scorePreloadSpecimen().andThen(actionEnd());
        intakeSpecimenCommand = autoCommand.autoIntakeSpecimen().andThen(actionEnd());
        scoreSpecimenCommand = autoCommand.autoScoreSpecimen().andThen(actionEnd());


        actions.addAll(Arrays.asList(scorePreloadCommand,
                null, null,
                null, null,
                null, null,
                null, null,
                intakeSpecimenCommand, null, scoreSpecimenCommand, null,
                intakeSpecimenCommand, null, scoreSpecimenCommand, null,
                intakeSpecimenCommand, null, scoreSpecimenCommand, null));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("drive error",follower.driveError);
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("follower finished",!follower.isBusy());
        telemetry.addData("action finished", !this.actionRunning);
        telemetry.addData("current path id", currentPathId);
        telemetry.addData("front arm", frontArm.state);
        telemetry.addData("lift arm", liftArm.state);
        /*telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());*/
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
            if(!follower.isBusy() && follower.driveError < 1.0 && !this.actionRunning){
                PathChain path = it.next();
                if(path!=null)follower.follow(path,1.2,1.8, Math.toRadians(10),1);
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
