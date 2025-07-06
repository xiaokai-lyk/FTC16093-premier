package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.utils.PathChainList;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Chamber", group = "Auto")
public class AutoChamber extends AutoOpModeEx {
    private Follower follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private FrontArm frontArm;
    private LiftArm liftArm;
    private Boolean actionRunning;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose0 = new Pose(1, 0, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(1, 0, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(1, 0, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(1, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(2, 0, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(9, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(6, 0, Math.toRadians(0));
    private final Pose HPPose = new Pose(2, 0, Math.toRadians(0));
    private final Pose parkControlPose = new Pose(3, 0,Math.toRadians(0));
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(0));
    private int currentPathId = 0;



    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
        liftArm.initPos();

        while(getRuntime()<2)CommandScheduler.getInstance().run();//进入init pos需要
    }

    private void buildPaths() {
        PathChain grabPickup1, grabPickup2, grabPickup3, goToHP1, goToHP2, goToHP3, scoreChamber0, scoreChamber1, scoreChamber2, scoreChamber3;
        scoreChamber0 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(scorePose0)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose0.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose0), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose0.getHeading(), pickup1Pose.getHeading())
                .build();

        goToHP1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(HPPose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), HPPose.getHeading())
                .build();

        scoreChamber1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HPPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(HPPose.getHeading(), scorePose1.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Pose.getHeading())
                .build();

        goToHP2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(HPPose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), HPPose.getHeading())
                .build();

        scoreChamber2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HPPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(HPPose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        goToHP3 = scoreChamber2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(HPPose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), HPPose.getHeading())
                .build();

        scoreChamber3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HPPose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(HPPose.getHeading(), scorePose3.getHeading())
                .build();

        Path park = new Path(new BezierCurve(new Point(scorePose3), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());

        pathChainList.addPath(null, scoreChamber0, grabPickup1, goToHP1, scoreChamber1, grabPickup2, goToHP2, scoreChamber2, grabPickup3, goToHP3, scoreChamber3);
    }

    private Command actionEnd(){
        return new InstantCommand(()->this.actionRunning = false);
    }

    private void buildActions(){
        Command intakePreloadFromHP, intakeSpecimenCommand, SpecimenHPCommand, scoreSpecimenCommand;
        intakePreloadFromHP = autoCommand.autointakePreloadSpecimen().andThen(actionEnd());
        intakeSpecimenCommand = autoCommand.autoIntakeSpecimen().andThen(actionEnd());
        SpecimenHPCommand = autoCommand.putSpecimenToHPCommand().andThen(autoCommand.autoIntakeSpecimen()).andThen(actionEnd());
        scoreSpecimenCommand = autoCommand.autoScoreSpecimen().andThen(actionEnd());


        actions.addAll(Arrays.asList(intakePreloadFromHP,scoreSpecimenCommand,
                intakeSpecimenCommand, SpecimenHPCommand, scoreSpecimenCommand,
                intakeSpecimenCommand, SpecimenHPCommand, scoreSpecimenCommand,
                intakeSpecimenCommand, SpecimenHPCommand, scoreSpecimenCommand));
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("follower finished",!follower.isBusy());
        telemetry.addData("action finished", !this.actionRunning);
        telemetry.addData("current path id", currentPathId);
        telemetry.addData("front arm", frontArm.state);
        telemetry.addData("lift arm", liftArm.state);
        telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());
        telemetry.addData("Current Path ID", currentPathId);
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
                if(path!=null)follower.followPath(path);
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
