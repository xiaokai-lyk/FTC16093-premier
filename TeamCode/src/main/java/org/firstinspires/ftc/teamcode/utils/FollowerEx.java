package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.pathgen.Path;

import java.util.Arrays;
import java.util.List;

public class FollowerEx extends Follower {
    private double xTolerance, yTolerance, headingTolerance;
    private final List<DcMotorEx> motors;
    private Point lastPoint;
    public boolean isFinished = true;

    public FollowerEx(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, FConstants, LConstants);
        DcMotorEx leftFront, leftRear, rightRear, rightFront;
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
    }

    public void follow(PathChain pathChain, double xTolerance, double yTolerance, double headingTolerance) {
        isFinished = false;
        super.followPath(pathChain);
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.headingTolerance = headingTolerance;
        Path lastPath = pathChain.getPath(pathChain.size()-1);
        lastPoint = lastPath.getPoint(lastPath.length());
    }

    @Override
    public void update(){
        super.update();
        if(!isFinished && Math.abs(super.getHeadingError()) < headingTolerance && super.atPoint(lastPoint, xTolerance, yTolerance)) {
            isFinished = true;
            super.breakFollowing();
            for (DcMotorEx motor : motors){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}
