//package org.firstinspires.ftc.teamcode.opmodes.autos;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
////import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoBasket.PathPoint.*;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
//import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;
//import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;
//import org.firstinspires.ftc.teamcode.opmodes.TeleOpBase;
//
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@Config
//@Autonomous(name = "Auto Basket", group = "Autos")
//public class AutoBasketMaster{
//    private static Follower follower;
//    private FrontArm frontArm;
//    private LiftArm liftArm;
//    private Timer pathTimer;
//    private int pathState;
//
//    public Path scorePreload;
//    public Path park;
//    public PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
//
//    public void autoInitialize() {
//        CommandScheduler.getInstance().cancelAll();
//        pathTimer = new Timer();
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//
//        frontArm = new FrontArm(hardwareMap);
//        liftArm = new LiftArm(hardwareMap);
//        frontArm.initPos();
//        liftArm.initPos();
//
//        buildPaths();
//        pathState = 0;
//    }
//
//    public void onStart() {
////        TeleOpBase auto = new TeleOpBase();
////        auto.onStart();
//    }
//
//    private void buildPaths() {
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
//    }
//
////    @Override
////    public void run() {
////        follower.update();
////        autonomousPathUpdate();
////
////        telemetry.addData("path state", pathState);
////        telemetry.addData("x", follower.getPose().getX());
////        telemetry.addData("y", follower.getPose().getY());
////        telemetry.addData("heading", follower.getPose().getHeading());
////        telemetry.update();
////    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scorePreload);
//                frontArm.handover().schedule();
//                liftArm.releaseHigh().schedule();
//                liftArm.releaseHigh().schedule();
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup1, true);
//                    frontArm.intake(true).schedule();
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup1, true);
//                    frontArm.handover().schedule();
//                    liftArm.releaseHigh().schedule();
//                    liftArm.releaseHigh().schedule();
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(park, true);
//                    setPathState(8);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void initialize() {
//        CommandScheduler.getInstance().cancelAll();
//        pathTimer = new Timer();
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);
//
//        frontArm = new FrontArm(hardwareMap);
//        liftArm = new LiftArm(hardwareMap);
//
//        buildPaths();
//        pathState = 0;
//    }
//}