package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.concurrent.atomic.AtomicReference;

import lombok.Getter;

public class SampleAutoAlignCommandPP extends CommandBase {
    private final Follower follower;
//    private final Vision vision;
    private final Telemetry telemetry;

    private AtomicReference<Double> turnServo;
    private AtomicReference<Double> slideExtension;

    private final double tickPerUnit = 422 / (440 / 25.4); // tick per inches
    private Path path;
    private boolean isTargetVisibleWhenStart = true;

    @Getter private boolean isInitializing = true;

    public SampleAutoAlignCommandPP(
            Follower follower,
//            Vision vision,
            Telemetry telemetry,
            AtomicReference<Double> turnServoSupplier,
            AtomicReference<Double> slideExtensionSupplier) {
        this.follower = follower;
//        this.vision = vision;
        this.telemetry = telemetry;
//        this.turnServo = turnServoSupplier;
//        this.slideExtension = slideExtensionSupplier;
    }

    @Override
    public void initialize() {
//        isInitializing = true;
//        isTargetVisibleWhenStart = vision.isTargetVisible();
//        if (isTargetVisibleWhenStart) {
//            Pose currentPose = follower.getPose();
//            Pose targetPose = vision.getTargetPose();
//
//            // 计算目标位置
//            double targetX = targetPose.getX();
//            double targetY = targetPose.getY();
//            double targetHeading = targetPose.getHeading();
//
//            // 设置伺服和滑轨位置
//            turnServo.set(calculateTurnServoPosition(targetHeading));
//            slideExtension.set(calculateSlideExtension(targetX, targetY));
//
//            // 创建路径
//            path = new Path(new BezierLine(new Point(currentPose), new Point(targetPose)));
//            path.setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading());
//
//            // 开始跟随路径
//            follower.followPath(path);
//        }
    }

    @Override
    public void execute() {
        if (isTargetVisibleWhenStart) {
            follower.update();
            telemetry.addData("AutoAlign", "Following path");
        } else {
            telemetry.addData("AutoAlign", "Target not visible");
        }
    }

    @Override
    public boolean isFinished() {
        if (!isTargetVisibleWhenStart) {
            return true;
        }
        boolean finished = !follower.isBusy();
        if (finished) {
            isInitializing = false;
        }
        return finished;
    }

    private double calculateTurnServoPosition(double targetHeading) {
        // 根据目标朝向计算伺服位置
        // 这里需要根据实际机器人的伺服角度范围进行调整
        return 0.5 + (targetHeading / (2 * Math.PI));
    }

    private double calculateSlideExtension(double targetX, double targetY) {
        // 根据目标位置计算滑轨延伸量
        // 这里需要根据实际机器人的滑轨范围进行调整
        double distance = Math.sqrt(targetX * targetX + targetY * targetY);
        return distance * tickPerUnit;
    }
}