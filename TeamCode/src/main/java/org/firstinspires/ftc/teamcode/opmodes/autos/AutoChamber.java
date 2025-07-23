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

//@Autonomous(name = "Auto Chamber", group = "Auto")
public class AutoChamber extends AutoOpModeEx {
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

    private final Pose pickup1Pose = new Pose(15, 23, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(15, 13, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(18, 11, Math.toRadians(-29));

    private final Pose HPPose = new Pose(0, 29, Math.toRadians(0));
    private final Pose HPPose1 = new Pose(2, 23, Math.toRadians(0));
    private final Pose HPPose2 = new Pose(2, 13, Math.toRadians(0));
    private final Pose HPPose3 = new Pose(0, 15, Math.toRadians(0));

    private final Pose scorePose0 = new Pose(27, 58, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(28, 60, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(28, 62, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(28, 64, Math.toRadians(0));

    private final Pose parkControlPose = new Pose(14, 22, Math.toRadians(0));
    private final Pose parkPose = new Pose(5, 22, Math.toRadians(0));
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
        PathChain grabPickup1, grabPickup2, grabPickup3,
                goToHP, goToHPAfterSample1, goToHPAfterSample2, goToHPAfterSample3,
                scoreChamber0, scoreChamber1, scoreChamber2, scoreChamber3, park;
        scoreChamber0 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose0)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose0.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose0), autoCommand.midPoint(scorePose0, pickup1Pose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose0.getHeading(), pickup1Pose.getHeading())
                .build();

        goToHP = follower.pathBuilder()
                .addPath(new BezierCurve(getCurrentPoint(), autoCommand.midPoint(follower.getPose(), HPPose), new Point(HPPose)))
                .setLinearHeadingInterpolation(getCurrentHeading(), HPPose.getHeading())
                .build();
        goToHPAfterSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(HPPose1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), HPPose1.getHeading())
                .build();
        goToHPAfterSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(HPPose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), HPPose2.getHeading())
                .build();
        goToHPAfterSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(HPPose3)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), HPPose3.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
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
                .addPath(new BezierCurve(new Point(scorePose3), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading())
                .build();

        pathChainList.addPath(scoreChamber0,
                grabPickup1, goToHPAfterSample1,
                grabPickup2, goToHPAfterSample2,
                grabPickup3, goToHPAfterSample3,
                null, scoreChamber1, null, goToHP,
                null, scoreChamber2, null, goToHP,
                null, scoreChamber3, null, park);
    }

    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakeSampleCommand, intakeFirstSpecimenCommand, intakeSpecimenCommand, giveSpecimenToHPCommand, scoreSpecimenCommand, scorePreloadCommand;
        scorePreloadCommand = autoCommand.scorePreloadSpecimen_v2().andThen(actionEnd());
        intakeSampleCommand = autoCommand.autoIntakeSampleForHP_v2().andThen(actionEnd());
        giveSpecimenToHPCommand = autoCommand.putSampleToHPCommand_v2().andThen(actionEnd());
        intakeFirstSpecimenCommand = autoCommand.autoIntakeFirstSpecimen_v2().andThen(actionEnd());
        intakeSpecimenCommand = autoCommand.autoIntakeSpecimen_v2().andThen(actionEnd());
        scoreSpecimenCommand = autoCommand.autoScoreSpecimen_v2().andThen(actionEnd());


        actions.addAll(Arrays.asList(scorePreloadCommand,
                intakeSampleCommand, giveSpecimenToHPCommand,
                intakeSampleCommand, giveSpecimenToHPCommand,
                intakeSampleCommand, giveSpecimenToHPCommand,
                intakeFirstSpecimenCommand, null, scoreSpecimenCommand, null,
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
            if(!follower.isBusy() && !this.actionRunning){
                PathChain path = it.next();
                if(path!=null)follower.follow(path,2,1.5, Math.toRadians(0));
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
