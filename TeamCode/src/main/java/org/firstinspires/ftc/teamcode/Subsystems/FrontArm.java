package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.SpinnerConstant;

import lombok.Getter;

@Getter
public class FrontArm {
    private Vision vision;
    private LLResult llResult;

    private final DcMotorEx frontSlide;

    private final Servo claw;
    private final Servo clawSpinner;
    private final Servo wrist;
    private final Servo armSpinner;
    private final Servo armWrist;

    private final AnalogInput claw_in;

    public SpinnerConstant currentSpinnerPos;
    public State state;
    public boolean claw_open = false;

    public enum State{
        FREE,
        DOWN,
        GIVEHP,
        HOLDING_BLOCK
    }


    public FrontArm(@NonNull HardwareMap hardwareMap){
        vision = new Vision(hardwareMap, null);
//        vision.initialize();
        this.armSpinner = hardwareMap.get(Servo.class, "armSpin");
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.wrist = hardwareMap.get(Servo.class, "wrist");
        this.clawSpinner = hardwareMap.get(Servo.class,"spin");
        this.armWrist = hardwareMap.get(Servo.class,"armWrist");
        this.frontSlide = hardwareMap.get(DcMotorEx.class, "slideFront");
        this.claw_in = hardwareMap.get(AnalogInput.class,"claw_in");
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getClawDeg(){
        return claw_in.getVoltage()/claw_in.getMaxVoltage()*360;
    }

    public void open_claw(boolean open){
        claw_open = open;
        if(claw_open){
            ServoConstants.CLAW_OPEN.setToServo(this.claw);
        }else{
            ServoConstants.CLAW_CLOSE.setToServo(this.claw);
        }
    }

    private boolean frontSlideFinished(){
        return frontSlide.getVelocity() < MotorConstants.FRONT_FINISH_THRESHOLD.value;
    }

    public void set_spinner(@NonNull SpinnerConstant pos){
        pos.setToServo(this.clawSpinner);
        this.currentSpinnerPos = pos;
    }
    public void set_wrist(@NonNull ServoConstants pos){
        pos.setToServo(this.wrist);
    }
    public void set_arm_wrist(@NonNull ServoConstants pos){
        pos.setToServo(this.armWrist);
    }
    public void set_arm_spinner(@NonNull ServoConstants pos){
        pos.setToServo(this.armSpinner);
    }
    public void set_arm_spinner(double servoPos){
        armSpinner.setPosition(servoPos);
    }

    public void setLED(boolean enable){
        vision.setLed(enable);
    }

    public boolean updateVision(){
        LLResult res = vision.getResult();
        if(vision.resultValid(res)){
            llResult = res;
            return true;
        }
        else return false;
    }

    public Command intakeWithVision(){
        return new SequentialCommandGroup(
                new InstantCommand(()->frontSlide.setTargetPosition(vision.getSlideTarget(llResult))),
                new WaitUntilCommand(this::frontSlideFinished),
                new InstantCommand(()->set_arm_spinner(vision.getArmSpinnerPos(llResult))),
                new WaitCommand(70),
                new InstantCommand(()->{
                    set_wrist(ServoConstants.WRIST_DOWN);
                    set_arm_wrist(ServoConstants.ARM_WRIST_DOWN);
                }),
                new WaitCommand(100),
                new InstantCommand(()-> open_claw(false)),
                new WaitCommand(50)
        );
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
            if(spinnerConstants[i]== currentSpinnerPos){
                if ((i==0 && !to_right) || (i == spinnerConstants.length-1 && to_right)){
                    continue;
                }
                set_spinner(spinnerConstants[(i+(to_right?1:-1))% spinnerConstants.length]);
                break;
            }
        }
    }

    public Command intake(boolean is_far, boolean auto_mode) {
        return new ConditionalCommand(
                new ConditionalCommand(
                        new InstantCommand(()-> {
                            set_wrist(ServoConstants.WRIST_DOWN);
                            set_arm_wrist(ServoConstants.ARM_WRIST_DOWN);
                        })
                                .andThen(
                                        new WaitCommand(70),
                                        new InstantCommand(()->ServoConstants.CLAW_CHECK.setToServo(claw)),
                                        new WaitCommand(70),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new InstantCommand(()->open_claw(false)),
                                                        new WaitCommand(50),
                                                        new InstantCommand(this::initPos),
                                                        new WaitCommand(120),
                                                        new InstantCommand(()->this.state = State.HOLDING_BLOCK)
                                                ),
                                                new InstantCommand(() ->
                                                {
                                                    open_claw(true);
                                                    frontSlide.setTargetPosition(is_far ? MotorConstants.FRONT_FAR.value : MotorConstants.FRONT_NEAR.value);
                                                    set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                                                    set_arm_wrist(ServoConstants.ARM_WRIST_PREINTAKE);
                                                    set_wrist(ServoConstants.WRIST_PARALLEL);
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
                                                ()->getClawDeg()>ServoConstants.CLAW_HAS_BLOCK_MIN_DEGREE.value || auto_mode
                                        )//检查有没有夹到块，若是自动模式则无论如何都夹起并交接
                                        //有块：夹起，没有：回intake状态
                                ),
                        new InstantCommand(()->this.frontSlide.setTargetPosition(is_far ? MotorConstants.FRONT_FAR.value : MotorConstants.FRONT_NEAR.value)),
                        ()->(is_far && (this.frontSlide.getCurrentPosition() >
                                0.97*MotorConstants.FRONT_FAR.value-MotorConstants.FRONT_TOLERANCE.value))
                                ||(
                                !is_far && this.frontSlide.getCurrentPosition() < MotorConstants.FRONT_NEAR.value + 10)
                        //判断is_far参数代表的滑轨位置和实际位置是否一致
                        //一致：夹起；不一致：动滑轨
                ),
                new InstantCommand(() ->

                {
                    this.state = State.DOWN;
                    open_claw(true);
                    frontSlide.setTargetPosition(is_far ? MotorConstants.FRONT_FAR.value : MotorConstants.FRONT_NEAR.value);
                    set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
                    set_arm_wrist(ServoConstants.ARM_WRIST_PREINTAKE);
                }).andThen(
                        new WaitUntilCommand(this::frontSlideFinished),
                        new InstantCommand(()->set_wrist(ServoConstants.WRIST_DOWN)),
                        new ConditionalCommand(
                                new InstantCommand(() -> set_spinner(SpinnerConstant.PARALLEL)),
                                new InstantCommand(),
                                () -> this.state != State.DOWN
                        )
                ),
                () -> (this.state == State.DOWN)//判断是不是已经放下小臂了
                //若没放下则放下小臂
        );
    }

    //Dual
    public Command createHandover(){
        return new InstantCommand(()->{
            set_wrist(ServoConstants.WRIST_HANDOVER);
            set_spinner(SpinnerConstant.PARALLEL);
            set_arm_wrist(ServoConstants.ARM_WRIST_HANDOVER);
            set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
            open_claw(false);
            frontSlide.setTargetPosition(0);
        }).andThen(
                new InstantCommand(()->this.state = State.FREE)
        );
    }
     public Command clawHandover(){
        return new SequentialCommandGroup(
                        new InstantCommand(()->this.open_claw(true)),
                        new InstantCommand(()->this.initPos(false)),
                        new InstantCommand(()->this.state = State.FREE)
                );

    }
    public  Command waitClawHandover(){
        return new SequentialCommandGroup(
                new InstantCommand(()->this.open_claw(true)),
                new WaitCommand(200),
                new InstantCommand(()->this.initPos(false)),
                new InstantCommand(()->this.state = State.FREE)

        );
    }

    public Command handover(){
        return new InstantCommand(()->{
            set_wrist(ServoConstants.WRIST_HANDOVER);
            set_spinner(SpinnerConstant.PARALLEL);
            set_arm_wrist(ServoConstants.ARM_WRIST_HANDOVER);
            set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
            open_claw(false);
            frontSlide.setTargetPosition(0);
        }).andThen(
                new WaitCommand(300),
                new InstantCommand(()->this.open_claw(true)),
                new WaitCommand(150),
                new InstantCommand(()->this.initPos(false)),
                new InstantCommand(()->this.state = State.FREE)
        );
    }
    public Command handover_1(){
        return new InstantCommand(()->{
            set_wrist(ServoConstants.WRIST_HANDOVER);
            set_spinner(SpinnerConstant.PARALLEL);
            set_arm_wrist(ServoConstants.ARM_WRIST_HANDOVER);
            set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
            open_claw(false);
            frontSlide.setTargetPosition(0);
        });
    }
    public Command handover_2(){
        return new SequentialCommandGroup(
                new WaitCommand(20),
                new InstantCommand(()->this.open_claw(true)),
                new WaitCommand(50),
                new InstantCommand(()->this.initPos(false)),
                new InstantCommand(()->this.state = State.FREE)
        );
    }

    public Command giveHP(){
        return new InstantCommand(()-> {
            this.state = State.GIVEHP;
            set_arm_spinner(ServoConstants.ARM_SPINNER_BACK);
            set_spinner(SpinnerConstant.GIVE_HP);
            set_wrist(ServoConstants.WRIST_PARALLEL);
            set_arm_wrist(ServoConstants.ARM_WRIST_TURN);
        });
    }

    public Command specimenToZone(){
        return new SequentialCommandGroup(
                new InstantCommand(()->open_claw(true)),
                new WaitCommand(200),
                new InstantCommand(this::initPos),
                new InstantCommand(()->this.state = State.FREE)
        );
    }

    public Command giveHPWithoutTurn(){
        return new SequentialCommandGroup(
                new InstantCommand(()->this.state = State.GIVEHP),
                new InstantCommand(()->frontSlide.setTargetPosition(MotorConstants.FRONT_FAR.value)),
                new WaitCommand(400),
                new InstantCommand(()->open_claw(true)),
                new WaitCommand(50),
                new InstantCommand(this::initPos),
                new InstantCommand(()->this.state = State.FREE)
        );
    }


    public void initPos(boolean resetSlide) {
        open_claw(claw_open);
        set_spinner(SpinnerConstant.PARALLEL);
        set_arm_wrist(ServoConstants.ARM_WRIST_CHAMBER_INTAKE);
        set_wrist(ServoConstants.WRIST_PARALLEL);
        set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
        this.state = State.FREE;
        if(resetSlide)resetSlide(); else frontSlide.setTargetPosition(0);
        frontSlide.setPower(1);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void initPos() {
        initPos(false);
    }

    public void autoInitPos(){
        open_claw(claw_open);
        set_spinner(SpinnerConstant.PARALLEL);
        set_arm_wrist(ServoConstants.ARM_WRIST_AUTO_FREE);
        set_wrist(ServoConstants.WRIST_PARALLEL);
        set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
        this.state = State.FREE;
        frontSlide.setTargetPosition(0);
        frontSlide.setPower(1);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoChamberInitPos(){
        open_claw(claw_open);
        set_spinner(SpinnerConstant.PARALLEL);
        set_arm_wrist(ServoConstants.ARM_WRIST_AUTOCHAMBER_FREE);
        set_wrist(ServoConstants.WRIST_PARALLEL);
        set_arm_spinner(ServoConstants.ARM_SPINNER_FRONT);
        this.state = State.FREE;
        frontSlide.setTargetPosition(0);
        frontSlide.setPower(1);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetSlide(){
        new SequentialCommandGroup(
                new InstantCommand(()->frontSlide.setTargetPosition(0)),
                new WaitUntilCommand(()->!frontSlide.isBusy()),
                new InstantCommand(()->frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)),
                new InstantCommand(()->frontSlide.setPower(-0.5)),
                new WaitCommand(100),
                new InstantCommand(()->frontSlide.setPower(0)),
                new InstantCommand(this::resetEncoder),
                new InstantCommand(()->frontSlide.setPower(1))
        ).schedule();
    }

    private void resetEncoder(){
        frontSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public Command highChamber(){
        return new InstantCommand(()->armWrist.setPosition(ServoConstants. ARM_WRIST_CHAMBER_INTAKE.value));
    }

    public void setPositionOffset(int offset){
        frontSlide.setTargetPosition(frontSlide.getTargetPosition() + offset);
    }
}