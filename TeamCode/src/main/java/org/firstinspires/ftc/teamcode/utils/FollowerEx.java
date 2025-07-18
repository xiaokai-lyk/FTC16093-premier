package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.sqrt;

import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.pathgen.Path;

import java.util.Arrays;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class FollowerEx extends Follower {
    private double xTolerance, yTolerance, headingTolerance;
    private final List<DcMotorEx> motors;
    private Point lastPoint;
    public boolean isFinished = true;
    private PinpointLocalizer pinpointLocalizer;
    private double minPowerScale;
    AutoCommand autoCommand;
    Follower follower;

    public FollowerEx(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants, PinpointLocalizer localizer) {
        super(hardwareMap, FConstants, LConstants);
        this.pinpointLocalizer = localizer;
        DcMotorEx leftFront, leftRear, rightRear, rightFront;
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);//That is not a good solution
        // , but it is the only solution think of.
    }

    public void followWithTolerance(FollowerEx follower, PathChain pathChain, double xTolerance, double yTolerance, double headingTolerance, double minPowerScale) {
        isFinished = false;
        super.followPath(pathChain);
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.headingTolerance = headingTolerance;
        this.minPowerScale = minPowerScale;
        this.follower = follower;
        Path lastPath = pathChain.getPath(pathChain.size()-1);
        lastPoint = lastPath.getPoint(lastPath.length());
    }

    @Override
    public void update(){
        super.update();

        if (!isFinished && lastPoint != null) {
            Pose currentPose = follower.getPose();
            Pose2d currentPose2d = new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
            Vector2d currentPos = currentPose2d.vec();
            Vector2d targetPos = new Vector2d(lastPoint.getX(), lastPoint.getY());
            double distance = targetPos.minus(currentPos).norm();

            double slowDownRange = sqrt(xTolerance*xTolerance + yTolerance*yTolerance);

            // 如果在减速范围内
            if (!isFinished && Math.abs(getHeadingError()) < headingTolerance && atPoint(lastPoint, xTolerance, yTolerance)) {
                double scale = minPowerScale + (1 - minPowerScale) * (distance / slowDownRange);
                scale = Math.max(scale, minPowerScale);

                // 对每个电机设置比例缩放功率
                for (DcMotorEx motor : motors) {
                    double rawPower = motor.getPower();
                    motor.setPower(rawPower * scale);
                }
            }
        }

        // 到达终点判断逻辑
        if(!isFinished && Math.abs(super.getHeadingError()) < headingTolerance*0.3 && super.atPoint(lastPoint, xTolerance*0.3, yTolerance*0.5)) {
            isFinished = true;
            super.breakFollowing();
            for (DcMotorEx motor : motors){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}
