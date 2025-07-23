/*
 * Author: Yunkai Li
 * Date: 2025/07/18
 * Intro: A wrapper of pedro pathing follower. Add tolerances to avoid swinging near the end point.
 * Please DO NOT delete this comment! Be respectful to the copyright!!!
 * */

package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.pathgen.BezierPoint;
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
    private Point endPoint;
    private double endHeading;
    public boolean isFinished = true;

    public FollowerEx(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, FConstants, LConstants);
        DcMotorEx leftFront, leftRear, rightRear, rightFront;
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);//That is not a good solution
        // , but it is the only solution think of.
    }

    public void follow(PathChain pathChain, double xTolerance, double yTolerance, double headingTolerance) {
        isFinished = false;

        super.setMaxPower(1);
        super.followPath(pathChain);
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
        this.headingTolerance = headingTolerance;
        Path lastPath = pathChain.getPath(pathChain.size()-1);
        endHeading = lastPath.getHeadingGoal(pathChain.size()-1);
        endPoint = lastPath.getPoint(lastPath.length());
    }

    @Override
    public void update(){
        super.update();
        if(!isFinished && Math.abs(super.getHeadingError()) < headingTolerance && super.atPoint(endPoint, xTolerance, yTolerance)) {
            isFinished = true;
            breakFollowing();
            holdPoint(new BezierPoint(endPoint), endHeading);
            super.setMaxPower(0.3);
            for (DcMotorEx motor : motors){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Stop following and brake when nearing the end point.
            }
        }
    }

    public void forceStop(){
        super.breakFollowing();
        for (DcMotorEx motor : motors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Stop following and brake when nearing the end point.
        }
    }
}