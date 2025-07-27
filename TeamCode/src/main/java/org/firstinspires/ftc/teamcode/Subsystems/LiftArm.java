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
        leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotor.setDirection(DcMotorEx.Direction.FORWARD);
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
                new InstantCommand(()->setPower(1))
//                new WaitUntilCommand(()->this.getPosition()>MotorConstants.FINAL_ASCENT_SLIDE_FINISH.value)
        );
    }

    void hold_slide(){
        setPosition(getPosition()-15);
    }

    void hold_slide_controlByPower(){
        setPower(-0.4);
    }

//    void release_slide(){
//        setPosition(getPosition()+20); //防止只有一侧爬升
//    }

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
    void setPosition(int pos){
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        leftMotor.setTargetPosition(pos);
        rightMotor.setTargetPosition(pos);
    }
    void setPower(double power){
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

    public Command lowBasketCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_LOW.value));
    }

    Command getFromWallCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_HIGH_CHAMBER.value));
    }

    Command FirstGetFromWallCommand(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_HIGH_CHAMBER_FIRST.value));
    }


    public void resetSlide(){
        new SequentialCommandGroup(
                new InstantCommand(()->setPosition(0)),
                new WaitUntilCommand(this::isFinished),
                new InstantCommand(()->setPower(-0.3)),
                new WaitCommand(70),
                new InstantCommand(()->setMode(DcMotor.RunMode.RUN_TO_POSITION)),
                new InstantCommand(this::resetEncoder),
                new InstantCommand(()->setPosition(0))
        ).schedule();
    }
    void resetSlideForChamber(){
        new SequentialCommandGroup(
                new InstantCommand(()->setPosition(0)),
                new WaitUntilCommand(this::isFinishedForSpecimen),
                new InstantCommand(()->setPower(-0.5)),
                new WaitCommand(50),
                new InstantCommand(()->setMode(DcMotor.RunMode.RUN_TO_POSITION)),
                new InstantCommand(()->setPosition(0)),
                new WaitUntilCommand(this::isFinishedForSpecimen),
                new InstantCommand(this::resetEncoder)
        ).schedule();
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
    public Command parkAim(){
        return new InstantCommand(()->setPosition(MotorConstants.LIFT_PARK_AIM.value));
    }

    public void disable(){
        setPower(0);
    }

    public boolean isFinished(){
        return isFinished(30);
    }
    public boolean isFinishedForSpecimen(){
        return isFinished(80);
    }

    public Command resetSlideForAutoChamberEnd(){
        return new SequentialCommandGroup(
                new InstantCommand(()->setPosition(0)),
                new WaitUntilCommand(this::isFinished),
                new InstantCommand(()->setPower(-0.3)),
                new WaitCommand(70),
                new InstantCommand(()->setMode(DcMotor.RunMode.RUN_TO_POSITION)),
                new InstantCommand(this::resetEncoder),
                new InstantCommand(()->setPosition(0))
        );
    }
}





@Getter
public class LiftArm {
    private final Lifter lifter;
    private final Servo clawUp, armUp, wristUp, ascentLeft, ascentRight;
    public LiftArmState state;
    public enum LiftArmState{
        WALL,//从墙上夹
        PRE_CHAMBER,//准备挂
        FREE,
        RELEASE
    }


    public LiftArm(HardwareMap hardwareMap) {
        this.lifter = new Lifter(hardwareMap);
        this.clawUp = hardwareMap.get(Servo.class, "clawUp");
        this.armUp = hardwareMap.get(Servo.class, "armUp");
        this.wristUp = hardwareMap.get(Servo.class, "wristUp");
        this.ascentLeft = hardwareMap.get(Servo.class, "ascentLeft");
        this.ascentRight = hardwareMap.get(Servo.class, "ascentRight");
        lifter.resetEncoder();
    }



    public void initPos(){
        ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_DOWN.value);
        ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_DOWN.value);
        lifter.resetSlide();
        clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value);
        armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
        wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
        this.state = LiftArmState.FREE;
    }
    public void initPosSpecimen(){
        ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_UP.value);
        ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_UP.value);
        lifter.resetSlide();
        clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE_CAN_SLIDE.value);
        armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
        wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
        this.state = LiftArmState.FREE;
    }

    public void autoInitPos(){
        ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_DOWN.value);
        ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_DOWN.value);
        clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value);
        armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
        wristUp.setPosition(ServoConstants.UP_WRIST_INIT.value);
    }

    public void autoChamberInitPos(){
        this.autoInitPos();
        this.state = LiftArmState.WALL;
    }

    public String slideInfo(){return lifter.getMotorInfo();}

    public Command highChamber(){
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            armUp.setPosition(ServoConstants.UP_ARM_WALL.value);
                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_WALL.value);
                            lifter.resetSlideForChamber();
                        }),
                        new WaitUntilCommand(lifter::isFinishedForSpecimen),
                        new InstantCommand(()->this.state=LiftArmState.WALL)
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value)),
                                new WaitCommand(30),
                                lifter.getFromWallCommand(),
                                new WaitUntilCommand(lifter::isFinishedForSpecimen),
                                new InstantCommand(()->{
                                    armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                                    wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                                    this.state = LiftArmState.PRE_CHAMBER;
                                })
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                                new WaitCommand(20),
                                new SequentialCommandGroup(
                                        new InstantCommand(lifter::resetSlideForChamber),
                                        new WaitUntilCommand(lifter::isFinishedForSpecimen),
                                        new InstantCommand(()->{
                                            armUp.setPosition(ServoConstants.UP_ARM_WALL.value);
                                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
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

    public Command FirstHighChamber(){
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(()->{
                            armUp.setPosition(ServoConstants.UP_ARM_WALL.value);
                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_WALL.value);
                            lifter.resetSlideForChamber();
                        }),
                        new WaitUntilCommand(lifter::isFinishedForSpecimen),
                        new InstantCommand(()->this.state=LiftArmState.WALL)
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value)),
                                new WaitCommand(30),
                                lifter.FirstGetFromWallCommand(),
                                new WaitUntilCommand(lifter::isFinishedForSpecimen),
                                new InstantCommand(()->{
                                    armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                                    wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                                    this.state = LiftArmState.PRE_CHAMBER;
                                })
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                                new WaitCommand(20),
                                new SequentialCommandGroup(
                                        new InstantCommand(lifter::resetSlideForChamber),
                                        new WaitUntilCommand(lifter::isFinishedForSpecimen),
                                        new InstantCommand(()->{
                                            armUp.setPosition(ServoConstants.UP_ARM_WALL.value);
                                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
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
            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
        }).andThen(
                new WaitCommand(400),
                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value))
        );
    }
    public Command handover_1(){
        return new InstantCommand(()->{
            armUp.setPosition(ServoConstants.UP_ARM_HANDOVER.value);
            wristUp.setPosition(ServoConstants.UP_WRIST_HANDOVER.value);
            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
        });
    }
    public Command handover_2(){
        return new SequentialCommandGroup(
                new WaitCommand(100),
                new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value))
        );
    }

    //Dual
    public Command createHandover(){
        return new InstantCommand(()->{
            armUp.setPosition(ServoConstants.UP_ARM_HANDOVER.value);
            wristUp.setPosition(ServoConstants.UP_WRIST_HANDOVER.value);
            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
        });
    }

    public Command afterHandover(){
        return new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value));
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
                                        })
                                ),
                                new WaitUntilCommand(lifter::isFinished),
                                new InstantCommand(()->this.state = LiftArmState.RELEASE)
                        ),
                new SequentialCommandGroup(
                        new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                        //new WaitCommand(100),
                        new InstantCommand(()->armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value)),
                        //new WaitCommand(150),
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

    public Command releaseLow(){
        return new ConditionalCommand(
                lifter.lowBasketCommand()
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()->lifter.isFinished(MotorConstants.LIFT_ABOVE_BASKET_TOLERANCE.value)),
                                        new InstantCommand(()->{
                                            armUp.setPosition(ServoConstants.UP_ARM_BASKET.value);
                                            wristUp.setPosition(ServoConstants.UP_WRIST_BASKET.value);
                                        })
                                ),
                                new WaitUntilCommand(lifter::isFinished),
                                new InstantCommand(()->this.state = LiftArmState.RELEASE)
                        ),
                new SequentialCommandGroup(
                        new InstantCommand(()->clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value)),
                        //new WaitCommand(100),
                        new InstantCommand(()->armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value)),
                        //new WaitCommand(150),
                        new InstantCommand(()->{
                            clawUp.setPosition(ServoConstants.UP_CLAW_OPEN.value);
                            armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                        }),
                        new InstantCommand(lifter::resetSlide),
                        new WaitUntilCommand(lifter::isFinished),
                        new InstantCommand(()->this.state = LiftArmState.FREE)
                ),
                ()->this.lifter.getPosition()<0.95*MotorConstants.LIFT_LOW.value
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

    public void hold_slide_controlByPower(){
        lifter.hold_slide_controlByPower();
    }

    public Command ascent_end(){
        return new SequentialCommandGroup(
                new InstantCommand(()->armUp.setPosition(ServoConstants.UP_ARM_BACK.value))
        );
    }
    public Command ascent_down() {
        return new InstantCommand(() -> {
            ascentLeft.setPosition(ServoConstants.ASCENT_LEFT_MID.value);
            ascentRight.setPosition(ServoConstants.ASCENT_RIGHT_MID.value);
        }).andThen(
                new WaitCommand(2000),
                lifter.ascent_down()
        );
    }
    public Command parkCommand() {
        return new SequentialCommandGroup(
                lifter.parkAim().alongWith(
                        new InstantCommand(()->{
                            armUp.setPosition(ServoConstants.UP_ARM_PARALLEL.value);
                            wristUp.setPosition(ServoConstants.UP_WRIST_PARALLEL.value);
                            clawUp.setPosition(ServoConstants.UP_CLAW_CLOSE.value);
                        })
                ),
                new WaitUntilCommand(lifter::isFinished),
                new WaitCommand(1000),
                new InstantCommand(()->lifter.setPosition(MotorConstants.LIFT_PARK.value)),
                new WaitUntilCommand(lifter::isFinished)
        );
    }


    public boolean isFinished(){
        return lifter.isFinished(15);
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

    public boolean lifterIsHigh(){
        return lifter.getPosition() > 0.5 * MotorConstants.LIFT_HIGH.value;
    }

    public boolean lifterIsLow(){
        return lifter.getPosition() > 0.3 * MotorConstants.LIFT_HIGH.value;
    }

    public void setLifterPower(double power){
        lifter.setPower(power);
    }

    public void resetLifterEncoderAndPower(){
        lifter.resetEncoder();
        lifter.setPower(-1);
    }

    public Command resetSlideForAutoChamberEnd(){
        return lifter.resetSlideForAutoChamberEnd();
    }
}