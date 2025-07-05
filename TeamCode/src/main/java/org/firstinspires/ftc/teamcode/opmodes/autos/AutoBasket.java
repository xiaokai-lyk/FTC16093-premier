package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
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

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Basket", group = "Auto")
public class AutoBasket extends AutoOpModeEx {
    private Follower follower;
    private AutoCommand autoCommand;
    private List<Command> actions;
    private FrontArm frontArm;
    private LiftArm liftArm;
    private Boolean actionEnded;
    private boolean pathStarted = false;


    private PathChainList pathChainList;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose scorePose = new Pose(1, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(2, 0, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(9, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(6, 0, Math.toRadians(0));
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
        this.actionEnded = false; // Initialize actionEnded

        buildPaths();
        buildActions();

        frontArm.initPos();
        liftArm.initPos();
    }

    private void buildPaths() {
        PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
        Path scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
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

        Path park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        // Create a "stay in place" path for the initial releaseHigh action
        PathChain stayInPlace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(startPose))) // Same start and end point
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();

        pathChainList.addPath(stayInPlace, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3);
        
        // Debug: Log the number of paths added
        telemetry.addData("Paths built", pathChainList.size());
        telemetry.update();
    }

    private Command actionEnd(){
        return new InstantCommand(()->{
            this.actionEnded = true;
            telemetry.addData("Action Ended", "Set to true at step " + currentPathId);
            telemetry.update();
        });
    }

    private void buildActions(){
        Command intakeCommand, releaseCommand, releaseCommand0;
        intakeCommand = autoCommand.autoSampleIntake().andThen(actionEnd());
        releaseCommand0 = autoCommand.autoReleaseHigh().andThen(actionEnd());
        releaseCommand = autoCommand.autoReleaseHigh().andThen(actionEnd());
        actions.addAll(Arrays.asList(releaseCommand0,
                intakeCommand, releaseCommand,
                intakeCommand, releaseCommand,
                intakeCommand, releaseCommand));
        
        // Debug: Log the number of actions added
        telemetry.addData("Actions built", actions.size());
        telemetry.update();
    }

    private void periodic() {
        CommandScheduler.getInstance().run();
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("lift slide info", liftArm.slideInfo());
        telemetry.addData("follower finished",!follower.isBusy());
        telemetry.addData("action finished", this.actionEnded);
        telemetry.addData("current path id", currentPathId);
        telemetry.addData("front arm", frontArm.state);
        telemetry.addData("lift arm", liftArm.state);
        telemetry.update();
    }

    @Override
    public void run() {
        telemetry.addData("Actions size", actions.size());
        telemetry.addData("PathChainList size", pathChainList.size());
        telemetry.addData("Current Path ID", currentPathId);
        telemetry.update();

        if (actions.size() < pathChainList.size()) {
            throw new IllegalStateException(
                    "Not enough actions (" + actions.size() +
                            ") for the number of paths (" + pathChainList.size() + ")"
            );
        }

        if (currentPathId > actions.size()) {
            telemetry.addData("Error", "currentPathId (" + currentPathId + ") >= actions.size (" + actions.size() + ")");
            telemetry.update();
            return;
        }

        while (currentPathId < pathChainList.size() && opModeIsActive()) {

            if (currentPathId >= actions.size()) {
                telemetry.addData("Error", "currentPathId out of bounds: " + currentPathId + " >= " + actions.size());
                telemetry.update();
                break;
            }

            Command currentAction = actions.get(currentPathId);

            PathChain currentPath = null;
            int pathIndex = 0;
            for (PathChain path : pathChainList) {
                if (pathIndex == currentPathId) {
                    currentPath = path;
                    break;
                }
                pathIndex++;
            }

            if (currentPath == null) {
                telemetry.addData("Error", "Could not find path for currentPathId: " + currentPathId);
                telemetry.update();
                break;
            }

            if (!currentAction.isScheduled() && !pathStarted) {
                currentAction.schedule();

                if (currentPathId != 0) { // 第一步不用启动路径
                    follower.followPath(currentPath);
                    pathStarted = true; // 只有路径启动后才标记为 started
                } else {
                    // 第一步不启动路径，直接认为路径已经结束
                    pathStarted = true; // 为了可以进入结束判断
                }

                this.actionEnded = false;

                telemetry.addData("Action Started", "Scheduled action and started path if needed");
                telemetry.update();
            }


            periodic();

            // 如果路径已经开始过，并且路径结束，动作结束，才可以进入下一步
            if (pathStarted && (!follower.isBusy() || currentPathId == 0) && (this.actionEnded || currentAction.isFinished())) {
                currentPathId++;
                pathStarted = false; // 下一步重新启动路径
                telemetry.addData("Step Complete", "Moving to next step: " + currentPathId);
                telemetry.update();
            }
        }
    }

}
