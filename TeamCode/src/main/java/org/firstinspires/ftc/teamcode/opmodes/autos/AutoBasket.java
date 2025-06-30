package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.Subsystems.SuperStructure;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Basket", group = "Auto")
public class AutoBasket extends AutoOpModeEx {
    private Follower follower;
    private AutoCommand autoCommand;
    private FrontArm frontArm;
    private LiftArm liftArm;
    private Timer pathTimer;
    private int pathState;

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose scorePose = new Pose(3, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(6, 0, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(9, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(6, 0, Math.toRadians(0));
    private final Pose parkControlPose = new Pose(3, 0,Math.toRadians(0));
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        frontArm = new FrontArm(hardwareMap);
        liftArm = new LiftArm(hardwareMap);
        buildPaths();
        pathState = 0;
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    @Override
    public void onStart() {
        pathTimer.resetTimer();
        pathState = 0;
        frontArm.initPos();
        liftArm.initPos();
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                schedule(new SequentialCommandGroup(liftArm.releaseHigh(), new ParallelCommandGroup(liftArm.releaseHigh()),new InstantCommand(frontArm::initPos)));
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    schedule(new SequentialCommandGroup(
                            autoCommand.autoIntake(true),
                            new WaitCommand(500),
                            frontArm.handover()));
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    liftArm.releaseHigh().schedule();
                    setPathState(3);
                }
                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup2, true);
//                    frontArm.intake(true).schedule();
//                    frontArm.intake(true).schedule();
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup2, true);
//                    frontArm.handover().schedule();
//                    liftArm.releaseHigh().schedule();
//                    liftArm.releaseHigh().schedule();
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup3, true);
//                    frontArm.intake(true).schedule();
//                    frontArm.intake(true).schedule();
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup3, true);
//                    frontArm.handover().schedule();
//                    liftArm.releaseHigh().schedule();
//                    liftArm.releaseHigh().schedule();
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//                    follower.followPath(park, true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
