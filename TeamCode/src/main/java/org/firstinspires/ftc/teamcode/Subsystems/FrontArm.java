package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.SpinnerConstant;

import java.util.ArrayList;
import java.util.List;


public class FrontArm extends SubsystemBase {
    private final DcMotorEx FrontSlide;

    private final Servo Claw;
    private final Servo ClawSpinner;
    private final Servo Wrist;
    private final Servo ArmSpinner;
    private final Servo ArmWrist;

    private final AnalogInput claw_in;

    public SpinnerConstant CurrentSpinnerPos;
    public State state;
    public boolean claw_open = false;

    public enum State{
        FREE,
        DOWN,
        GIVEHP,
        HANDOVER
    }


    public FrontArm(HardwareMap hardwareMap){
        this.ArmSpinner = hardwareMap.get(Servo.class, "armSpin");
        this.Claw = hardwareMap.get(Servo.class, "claw");
        this.Wrist = hardwareMap.get(Servo.class, "wrist");
        this.ClawSpinner = hardwareMap.get(Servo.class,"spin");
        this.ArmWrist = hardwareMap.get(Servo.class,"armWrist");
        this.FrontSlide = hardwareMap.get(DcMotorEx.class, "slideFront");
        this.claw_in = hardwareMap.get(AnalogInput.class,"claw_in");
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getClawDeg(){
        return claw_in.getVoltage()/claw_in.getMaxVoltage()*360;
    }

    public void open_claw(boolean open){
        claw_open = open;
        if(claw_open){
            ServoConstants.CLAW_OPEN.setToServo(this.Claw);
        }else{
            ServoConstants.CLAW_CLOSE.setToServo(this.Claw);
        }
    }

    private void set_spinner(@NonNull SpinnerConstant pos){
        pos.setToServo(this.ClawSpinner);
        this.CurrentSpinnerPos = pos;
    }
    private void set_wrist(@NonNull ServoConstants pos){
        pos.setToServo(this.Wrist);
    }
    private void set_arm_wrist(@NonNull ServoConstants pos){
        pos.setToServo(this.ArmWrist);
    }
    private void set_arm_spinner(@NonNull ServoConstants pos){
        pos.setToServo(this.ArmSpinner);
    }

    public void spinner_rotate(boolean to_right){
        SpinnerConstant[] spinnerConstants={
                SpinnerConstant.DEG1,
                SpinnerConstant.DEG2,
                SpinnerConstant.DEG3,
                SpinnerConstant.PARALLEL,
                SpinnerConstant.DEG4,
                SpinnerConstant.DEG5,
        };
        for(int i=0;i< spinnerConstants.length;i++){
            if(spinnerConstants[i]==CurrentSpinnerPos){
                if ((i==0 && !to_right) || (i == spinnerConstants.length-1 && to_right)){
                    continue;
                }
                set_spinner(spinnerConstants[(i+(to_right?1:-1))% spinnerConstants.length]);
                break;
            }
        }
    }

    public Command intake(boolean is_far) {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new InstantCommand(()-> {
                            set_wrist(ServoConstants.WRIST_DOWN);
                            set_arm_wrist(ServoConstants.ARM_WRIST_DOWN);
                        })
                                .andThen(
                                        new WaitCommand(30),
                                        new InstantCommand(()->ServoConstants.CLAW_CHECK.setToServo(Claw)),
                                        new WaitCommand(70),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new InstantCommand(()->open_claw(false)),
                                                        new WaitCommand(50),
                                                        new InstantCommand(this::initPos),
                                                        new WaitCommand(120),
                                                        new InstantCommand(()->this.state = State.HANDOVER)
                                                ),
                                                new InstantCommand(() ->
                                                {
                                                    open_claw(true);
                                                    FrontSlide.setTargetPosition(is_far ? MotorConstants.FRONT_MAX.value : MotorConstants.FRONT_NEAR.value);
                                                    set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                                                    set_arm_wrist(ServoConstants.ARM_WRIST_PREINTAKE);
                                                    set_wrist(ServoConstants.WRIST_DOWN);
                                                    this.state = State.DOWN;
                                                }).andThen(
                                                        new ConditionalCommand(
                                                                new InstantCommand(() -> set_spinner(SpinnerConstant.PARALLEL)),
                                                                new InstantCommand(),
                                                                () -> this.state != State.DOWN
                                                        ),
                                                        new InstantCommand(() -> this.state = State.DOWN)
                                                ),
                                                ()->getClawDeg()>ServoConstants.CLAW_HAS_BLOCK_MIN_DEGREE.value
                                        )//检查有没有夹到块，
                                        //有块：夹起，没有：回intake状态
                                ),
                        new InstantCommand(()->this.FrontSlide.setTargetPosition(is_far ? MotorConstants.FRONT_MAX.value : MotorConstants.FRONT_NEAR.value)),
                        ()->(is_far && (this.FrontSlide.getCurrentPosition() >
                                0.97*MotorConstants.FRONT_MAX.value-MotorConstants.FRONT_TOLERANCE.value))
                                ||(
                                !is_far && this.FrontSlide.getCurrentPosition() < MotorConstants.FRONT_NEAR.value + 10
                        )//判断is_far参数代表的滑轨位置和实际位置是否一致
                        //一致：夹起；不一致：动滑轨
                ),
                new InstantCommand(() ->
                {
                    open_claw(true);
                    FrontSlide.setTargetPosition(is_far ? MotorConstants.FRONT_MAX.value : MotorConstants.FRONT_NEAR.value);
                    set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                    set_arm_wrist(ServoConstants.ARM_WRIST_PREINTAKE);
                    set_wrist(ServoConstants.WRIST_DOWN);
                    this.state = State.DOWN;
                }).andThen(
                        new ConditionalCommand(
                                new InstantCommand(() -> set_spinner(SpinnerConstant.PARALLEL)),
                                new InstantCommand(),
                                () -> this.state != State.DOWN
                        ),
                        new InstantCommand(() -> this.state = State.DOWN)
                ),
                () -> (this.state == State.DOWN)//判断是不是已经放下小臂了
                //若没放下则放下小臂
        );
    }

    public Command handover(){
        return new InstantCommand(()->{
            set_wrist(ServoConstants.WRIST_HANDOVER);
            set_spinner(SpinnerConstant.PARALLEL);
            set_arm_wrist(ServoConstants.ARM_WRIST_HANDOVER);
            set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
            open_claw(false);
            FrontSlide.setTargetPosition(0);
        }).andThen(
                new WaitCommand(300),
                new InstantCommand(()->this.open_claw(true)),
                new WaitCommand(200),
                new InstantCommand(this::initPos),
                new InstantCommand(()->this.state = State.FREE)
        );
    }

    public Command giveHP(){
        return new InstantCommand(()-> {
            this.state = State.GIVEHP;
            set_arm_spinner(ServoConstants.ARM_SPINNER_BACK);
            set_spinner(SpinnerConstant.PARALLEL);
            set_wrist(ServoConstants.WRIST_PARALLEL);
            set_arm_wrist(ServoConstants.ARM_WRIST_TURN);
        }).andThen(
                new WaitCommand(300),
                new InstantCommand(()->{
                    open_claw(true);
                }),
                new WaitCommand(300),
                new InstantCommand(this::initPos),
                new InstantCommand(()->this.state = State.FREE)
        );
    }

    public void initPos() {
        open_claw(claw_open);
        set_spinner(SpinnerConstant.PARALLEL);
        set_arm_wrist(ServoConstants.ARM_WRIST_FREE);
        set_wrist(ServoConstants.WRIST_PARALLEL);
        set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
        this.state = State.FREE;

        FrontSlide.setTargetPosition(0);
        FrontSlide.setPower(1);
        FrontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
