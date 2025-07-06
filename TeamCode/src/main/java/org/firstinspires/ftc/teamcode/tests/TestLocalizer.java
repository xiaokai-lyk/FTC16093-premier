package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp (name = "Test Localizer", group = "Testing")
@Config
public class TestLocalizer extends LinearOpMode {
    Follower follower;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    // Dashboard configurable parameters
    @Config
    public static class MovementConfig {
        public static double startX = -15;
        public static double startY = 62.3;
        public static double startHeading = 90;

        public static double targetX = 24;
        public static double targetY = 0;
        public static double targetHeading = 0;

        public static double moveToleranceX = 2.0;
        public static double moveToleranceY = 2.0;
        public static double moveToleranceHeading = 5.0;
    }

    private Pose2d startPos;
    private boolean isMoving = false;
    private long moveStartTime = 0;
    private static final long MOVE_TIMEOUT_MS = 10000; // 10 second timeout
    @Override
    public void runOpMode(){
        // Initialize Follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Set initial position
        startPos = new Pose2d(MovementConfig.startX, MovementConfig.startY, Math.toRadians(MovementConfig.startHeading));
        follower.setStartingPose(new Pose(MovementConfig.startX, MovementConfig.startY, Math.toRadians(MovementConfig.startHeading)));
        follower.update();

        // Setup telemetry
        telemetry_M.addData("Status", "Initialized");
        telemetry_M.addData("Start Position", "X: %.2f, Y: %.2f, Heading: %.2f",
            MovementConfig.startX, MovementConfig.startY, MovementConfig.startHeading);
        telemetry_M.addData("Target Position", "X: %.2f, Y: %.2f, Heading: %.2f",
            MovementConfig.targetX, MovementConfig.targetY, MovementConfig.targetHeading);
        telemetry_M.addData("Controls", "A: Move to target, B: Stop, X: Reset position");
        telemetry_M.update();

        waitForStart();

        while (opModeIsActive()){
            // Update follower
            follower.update();

            // Get current position
            Pose currentPose = follower.getPose();
            if (currentPose == null) {
                telemetry_M.addData("ERROR", "Current pose is null!");
                telemetry_M.update();
                sleep(20);
                continue;
            }
            double currentX = currentPose.getX();
            double currentY = currentPose.getY();
            double currentHeading = Math.toDegrees(currentPose.getHeading());

            // Handle input controls
            if (gamepad1.a && !isMoving) {
                // Move to target position
                Pose robotPose = follower.getClosestPose();
                if (robotPose == null) {
                    telemetry_M.addData("ERROR", "Robot pose is null - cannot start movement!");
                    telemetry_M.update();
                    sleep(20);
                    continue;
                }
                Point startPoint = new Point(robotPose.getX(), robotPose.getY(), Point.CARTESIAN);
                Point targetPoint = new Point(MovementConfig.targetX, MovementConfig.targetY, Point.CARTESIAN);

                Path movePath = new Path(new BezierLine(startPoint, targetPoint));
                movePath.setLinearHeadingInterpolation(robotPose.getHeading(), Math.toRadians(MovementConfig.targetHeading));

                follower.followPath(movePath);
                isMoving = true;
                moveStartTime = System.currentTimeMillis();
                telemetry_M.addData("Action", "Moving to target position");
            }

            if (gamepad1.b) {
                // Stop movement by creating a path to current position
                Pose stopPose = follower.getClosestPose();
                if (stopPose == null) {
                    telemetry_M.addData("ERROR", "Stop pose is null - cannot stop movement!");
                    telemetry_M.update();
                    sleep(20);
                    continue;
                }
                Point currentPoint = new Point(stopPose.getX(), stopPose.getY(), Point.CARTESIAN);
                Path stopPath = new Path(new BezierLine(currentPoint, currentPoint));
                stopPath.setConstantHeadingInterpolation(stopPose.getHeading());
                follower.followPath(stopPath);
                isMoving = false;
                telemetry_M.addData("Action", "Movement stopped");
            }

            if (gamepad1.x) {
                // Reset to start position
                follower.setStartingPose(new Pose(MovementConfig.startX, MovementConfig.startY, Math.toRadians(MovementConfig.startHeading)));
                follower.update();
                isMoving = false;
                telemetry_M.addData("Action", "Position reset to start");
            }

            // Check if movement is complete
            if (isMoving) {
                if (!follower.isBusy()) {
                    isMoving = false;
                    telemetry_M.addData("Action", "Movement completed");
                } else if (System.currentTimeMillis() - moveStartTime > MOVE_TIMEOUT_MS) {
                    // Timeout - stop movement by creating a path to current position
                    Pose timeoutPose = follower.getClosestPose();
                    if (timeoutPose == null) {
                        telemetry_M.addData("ERROR", "Timeout pose is null - cannot handle timeout!");
                        telemetry_M.update();
                        sleep(20);
                        continue;
                    }
                    Point timeoutPoint = new Point(timeoutPose.getX(), timeoutPose.getY(), Point.CARTESIAN);
                    Path timeoutPath = new Path(new BezierLine(timeoutPoint, timeoutPoint));
                    timeoutPath.setConstantHeadingInterpolation(timeoutPose.getHeading());
                    follower.followPath(timeoutPath);
                    isMoving = false;
                    telemetry_M.addData("Action", "Movement timeout");
                }
            }

            // Calculate error - with null checks
            Pose closestPose = follower.getClosestPose();
            double errorX = 0, errorY = 0, errorHeading = 0;

            if (closestPose != null && follower.poseUpdater != null) {
                Pose poseUpdaterPose = follower.poseUpdater.getPose();
                if (poseUpdaterPose != null) {
                    errorX = closestPose.getX() - poseUpdaterPose.getX();
                    errorY = closestPose.getY() - poseUpdaterPose.getY();
                    errorHeading = follower.getHeadingError();
                } else {
                    telemetry_M.addData("ERROR", "Pose updater pose is null!");
                }
            } else {
                telemetry_M.addData("ERROR", "Closest pose or pose updater is null!");
            }

            // Display telemetry
            telemetry_M.addData("Current Position", "X: %.3f, Y: %.3f, Heading: %.2f°", currentX, currentY, currentHeading);
            telemetry_M.addData("Target Position", "X: %.3f, Y: %.3f, Heading: %.2f°", MovementConfig.targetX, MovementConfig.targetY, MovementConfig.targetHeading);
            telemetry_M.addData("Error", "X: %.3f, Y: %.3f, Heading: %.2f°", errorX, errorY, errorHeading);
            telemetry_M.addData("Movement Status", isMoving ? "Moving" : "Idle");
            telemetry_M.addData("Follower Busy", follower.isBusy());
            telemetry_M.addData("Move Timeout", "%.1fs", (System.currentTimeMillis() - moveStartTime) / 1000.0);
            telemetry_M.update();

            // Small delay to prevent overwhelming the system
            sleep(20);
        }
    }
}
