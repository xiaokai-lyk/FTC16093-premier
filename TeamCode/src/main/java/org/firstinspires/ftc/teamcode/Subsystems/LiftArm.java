package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;

import lombok.Getter;

class Lifter{
    @Getter
    private final DcMotorEx leftMotor, rightMotor;
    private final Servo shifter;
    private DcMotorEx.RunMode mode;
    public Lifter(@NonNull HardwareMap hardwareMap){
        this.leftMotor = hardwareMap.get(DcMotorEx.class, "slideLeft");
        this.rightMotor = hardwareMap.get(DcMotorEx.class, "slideRight");
        this.shifter = hardwareMap.get(Servo.class, "shifter");

        this.mode = DcMotorEx.RunMode.RUN_TO_POSITION;

        shifter.setPosition(ServoConstants.SHIFTER_NORMAL.value);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);
        leftMotor.setMode(this.mode);
        rightMotor.setMode(this.mode);
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }

    private void toSlowMode(){
        shifter.setPosition(ServoConstants.SHIFTER_SLOW.value);
    }

    public Command ascent_up(){
        return new SequentialCommandGroup(
                new InstantCommand(()->setPower(0.5)),
                new InstantCommand(this::toSlowMode),
                new WaitUntilCommand(()->this.getPosition()>MotorConstants.FINAL_ASCENT_THRESHOLD.value),
                new InstantCommand(()->setPower(0.7))
        );
    }

    void hold_slide(){
        setPosition(getPosition()-10);
    }

    public Command ascent_down(){
        return new InstantCommand(()->setPower(-1));
    }

    void setMode(DcMotorEx.RunMode new_mode){
        if(mode!=new_mode){
            leftMotor.setMode(new_mode);
            rightMotor.setMode(new_mode);
            this.mode = new_mode;
        }
    }
    public void setPosition(int pos){
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition(pos);
        rightMotor.setTargetPosition(pos);
    }
    private void setPower(double power){
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    @SuppressLint("DefaultLocale")
    String getMotorInfo(){
        return String.format("Current Pos: %d %d, Target Pos: %d %d, \nError %d %d, Finished %b, Power: %.2f %.2f, Working: %b %b",
                leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition(),
                leftMotor.getTargetPosition(), rightMotor.getTargetPosition(),
                Math.abs(leftMotor.getCurrentPosition()- leftMotor.getTargetPosition()),
                Math.abs(rightMotor.getCurrentPosition()- rightMotor.getTargetPosition()),
                isFinished(),
                leftMotor.getPower(), leftMotor.getPower(),
                leftMotor.isBusy(), rightMotor.isBusy()
                );
    }

    public Command highBasketCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_HIGH.value));
    }

    Command getFromWallCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_HIGH_CHAMBER.value));
    }


    void resetSlide(){
        setPosition(0);
    }
    void resetEncoder(){
        setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public int getPosition(){
        return (leftMotor.getCurrentPosition()+ rightMotor.getCurrentPosition())/2;
    }
    public boolean isFinished(int tolerance) {
        return Math.abs((leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2
                - ((leftMotor.getTargetPosition() + rightMotor.getTargetPosition()) / 2)) < tolerance;
    }
    public boolean isFinished(){
        return isFinished(15);
    }
}





@Getter
public class LiftArm {
    private final Lifter lifter;
    private final Servo clawUp, armUp, wristUp, slideUp, ascentLeft, ascentRight;
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
        this.ascentLeft = hardwareMap.get(Servo.class, "ascentLeft");
        this.ascentRight = hardwareMap.get(Servo.class, "ascentRight");
        lifter.resetEncoder();
    }



    public void initPos(){
        new SequentialCommandGroup(
                new InstantCommand(()->slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value)),
                new WaitCommand(70),
                new InstantCommand(()->{
                    ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_DOWN.value);
                    ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_DOWN.value);
                    lifter.resetSlide();
                    clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE_CAN_SLIDE.value);
                    armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                    wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                    this.state = LiftArmState.FREE;
                })
        ).schedule();
    }

    public String slideInfo(){return lifter.getMotorInfo();}

    public Command highChamber(){
        return new ConditionalCommand(
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
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value)),
                                new WaitCommand(50),
                                lifter.getFromWallCommand(),
                                new WaitUntilCommand(lifter::isFinished),
                                new InstantCommand(()->{
                                    armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                                    wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                                }),
                                new WaitCommand(150),
                                new InstantCommand(()->{
                                    slideUp.setPosition(ServoConstants.UP_SLIDE_MAX.value);
                                    this.state = LiftArmState.PRE_CHAMBER;
                                })
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                                new WaitCommand(70),
                                new InstantCommand(()->slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value)),
                                new SequentialCommandGroup(
                                        new InstantCommand(()->slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value)),
                                        new WaitCommand(70),
                                        new InstantCommand(lifter::resetSlide),
                                        new WaitUntilCommand(lifter::isFinished),
                                        new InstantCommand(()->{
                                            armUp.setPosition(ServoConstants.UP_ARM_WALL.value);
                                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                                            slideUp.setPosition(ServoConstants.UP_SLIDE_MIN.value);
                                            wristUp.setPosition(ServoConstants.UP_WRIST_WALL.value);
                                            this.state=LiftArmState.WALL;
                                        })
                                )
                        )
                        ,()->this.state==LiftArmState.WALL
                ),
                ()->this.state==LiftArmState.FREE
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
                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE_CAN_SLIDE.value))
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
                        new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value)),
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

    public Command ascent_up(){
        return lifter.ascent_up().alongWith(
                new InstantCommand(()->{
                    ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_UP.value);
                    ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_UP.value);
                    armUp.setPosition(ServoConstants.UP_ARM_UPWARD.value);
                })
        );
    }

    public void hold_slide(){
        lifter.hold_slide();
    }

    public Command ascent_end(){
        return new SequentialCommandGroup(
                new InstantCommand(()->armUp.setPosition(ServoConstants.UP_ARM_BACK.value)),
                new WaitCommand(150),
                new InstantCommand(()->slideUp.setPosition(ServoConstants.UP_SLIDE_MAX.value))
        );
    }
    public Command ascent_down() {
        return new InstantCommand(() -> {
            ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_DOWN.value);
            ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_DOWN.value);
        }).andThen(
                new WaitCommand(700),
                lifter.ascent_down()
        );
    }

    public void setPosition(int pos){
        lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lifter.getLeftMotor().setTargetPosition(pos);
        lifter.getRightMotor().setTargetPosition(pos);
    }
    public void resetSlide(){
        setPosition(0);
    }
    public int getPosition(){
        return (lifter.getLeftMotor().getCurrentPosition()+lifter.getRightMotor().getCurrentPosition())/2;
    }
    public boolean isFinished(int tolerance) {
        return lifter.isFinished(tolerance);
    }
    public boolean isFinished(){
        return isFinished(15);
    }
    @Deprecated
    public Command moveSlideUpTo(int targetPosition, double power, int tolerance) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // 设置左电机
                    lifter.getLeftMotor().setTargetPosition(targetPosition);
                    lifter.getLeftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter.getLeftMotor().setPower(power);

                    // 设置右电机
                    lifter.getRightMotor().setTargetPosition(targetPosition);
                    lifter.getRightMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter.getRightMotor().setPower(power);
                }),
                new WaitUntilCommand(() ->
                        Math.abs(lifter.getLeftMotor().getCurrentPosition() - targetPosition) <= tolerance
                                &&
                                Math.abs(lifter.getRightMotor().getCurrentPosition() - targetPosition) <= tolerance
                )
        );
    }
    public Command reachHighBasket(int tolerance){
//        return moveSlideUpTo(MotorConstants.LIFT_HIGH.value, 0.95, tolerance);
        return lifter.highBasketCommand();
    }


}