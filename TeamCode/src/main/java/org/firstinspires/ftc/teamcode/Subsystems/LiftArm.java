package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;

class Lifter{
    private final DcMotorEx LeftMotor, RightMotor;
    private final Servo shifter;
    private DcMotorEx.RunMode mode;
    public Lifter(HardwareMap hardwareMap){
        this.LeftMotor = hardwareMap.get(DcMotorEx.class, "slideLeft");
        this.RightMotor = hardwareMap.get(DcMotorEx.class, "slideRight");
        this.shifter = hardwareMap.get(Servo.class, "shifter");

        this.mode = DcMotorEx.RunMode.RUN_TO_POSITION;

        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setTargetPosition(0);
        RightMotor.setTargetPosition(0);
        LeftMotor.setMode(this.mode);
        RightMotor.setMode(this.mode);
        LeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        RightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LeftMotor.setPower(1);
        RightMotor.setPower(1);
    }

    private void setMode(DcMotorEx.RunMode new_mode){
        if(mode!=new_mode){
            LeftMotor.setMode(new_mode);
            RightMotor.setMode(new_mode);
            this.mode = new_mode;
        }
    }
    private void setPosition(int pos){
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LeftMotor.setTargetPosition(pos);
        RightMotor.setTargetPosition(pos);
    }
    private void setPower(double power){
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LeftMotor.setPower(power);
        RightMotor.setPower(power);
    }

    @SuppressLint("DefaultLocale")
    String getMotorInfo(){
        return String.format("Current Pos: %d %d, Target Pos: %d %d, \nError %d %d, Finished %b, Working: %b %b",
                LeftMotor.getCurrentPosition(),RightMotor.getCurrentPosition(),
                LeftMotor.getTargetPosition(),RightMotor.getTargetPosition(),
                Math.abs(LeftMotor.getCurrentPosition()-LeftMotor.getTargetPosition()),
                Math.abs(RightMotor.getCurrentPosition()-RightMotor.getTargetPosition()),
                isFinished(),LeftMotor.isBusy(), RightMotor.isBusy()
                );
    }

    Command highBasketCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_HIGH.value));
    }

    Command getFromWallCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_FROM_WALL.value));
    }


    void resetSlide(){
        setPosition(0);
    }
    void resetEncoder(){
        setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public int getPosition(){
        return (LeftMotor.getCurrentPosition()+RightMotor.getCurrentPosition())/2;
    }
    public boolean isFinished(int tolerance) {
        return Math.abs((LeftMotor.getCurrentPosition() + RightMotor.getCurrentPosition()) / 2
                - ((LeftMotor.getTargetPosition() + RightMotor.getTargetPosition()) / 2)) < tolerance;
    }
    public boolean isFinished(){
        return isFinished(15);
    }
}






public class LiftArm extends SubsystemBase {
    private final Lifter lifter;
    private final Servo clawUp, armUp, wristUp, slideUp;
    public LiftArmState state;
    public enum LiftArmState{
        WALL,//从墙上夹
        PRE_CHAMBER,//准备挂
        FREE
    }


    public LiftArm(HardwareMap hardwareMap) {
        this.lifter = new Lifter(hardwareMap);
        this.clawUp = hardwareMap.get(Servo.class, "clawUp");
        this.armUp = hardwareMap.get(Servo.class, "armUp");
        this.wristUp = hardwareMap.get(Servo.class, "wristUp");
        this.slideUp = hardwareMap.get(Servo.class, "slideUp");
        lifter.resetEncoder();
    }



    public void initPos(){
        lifter.resetSlide();
        clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value);
        armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
        wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
        slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value);
        this.state = LiftArmState.FREE;
    }

    public String slideInfo(){return lifter.getMotorInfo();}

    public Command getFromWall(){
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value)),
                        new WaitCommand(30),
                        lifter.getFromWallCommand(),
                        new WaitUntilCommand(lifter::isFinished),
                        new InstantCommand(()->{
                            clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value);
                            armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                            slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value);
                            this.state = LiftArmState.PRE_CHAMBER;
                        })//TODO: replace this with correct values!

                ),
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            armUp.setPosition(ServoConstants.UP_ARM_WALL.value);
                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                            slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_WALL.value);
                            lifter.resetSlide();
                        }),
                        new WaitUntilCommand(lifter::isFinished),
                        new InstantCommand(()->this.state=LiftArmState.WALL)
                )
                ,
                ()->this.state==LiftArmState.WALL
        );
    }

    public Command handover(){
        return new InstantCommand(()->{
            armUp.setPosition(ServoConstants.UP_ARM_HANDOVER.value);
            wristUp.setPosition(ServoConstants.UP_WRIST_HANDOVER.value);
            slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value);
            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
        }).andThen(
                new WaitCommand(400),
                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value))
        );
    }

    public Command releaseHigh(){
        return new ConditionalCommand(
                lifter.highBasketCommand()
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->lifter.isFinished(MotorConstants.LIFT_ABOVE_BASKET_TOLERANCE.value)),
                                        new InstantCommand(()->{
                                            armUp.setPosition(ServoConstants.UP_ARM_BASKET.value);
                                            wristUp.setPosition(ServoConstants.UP_WRIST_BASKET.value);
                                        }),
                                        new WaitCommand(100),
                                        new InstantCommand(()->slideUp.setPosition(ServoConstants.UP_SLIDE_MAX.value))
                                ),
                                new WaitUntilCommand(lifter::isFinished)
                        ),
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                        }),
                        new WaitCommand(100),
                        new InstantCommand(()->armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value)),
                        new WaitCommand(150),
                        new InstantCommand(()->slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value)),
                        new WaitCommand(100),
                        new InstantCommand(()->{
                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                            armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                        }),
                        new InstantCommand(lifter::resetSlide),
                        new WaitUntilCommand(lifter::isFinished),
                        new InstantCommand(()->this.state = LiftArmState.FREE)
                ),
                ()->this.lifter.getPosition()<0.95*MotorConstants.LIFT_HIGH.value
        );
    }
}