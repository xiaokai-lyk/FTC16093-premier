package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Subsystems.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Constants.SpinnerConstant;

public class SuperStructure {
    private static SuperStructure instance;

    public DcMotorEx slideLeft, slideRight;
    public Servo shifter;
    public Servo clawUp, armUp, wristUp, slideUp;

    public DcMotorEx slideFront;
    public Servo claw, spin, wrist, armSpin, armWrist;

    public Servo led;

    private Servo Claw;
    private Servo ClawSpinner;
    private Servo Wrist;
    private Servo ArmSpinner;
    private Servo ArmWrist;

    private AnalogInput claw_in;

    public SpinnerConstant CurrentSpinnerPos;
    public boolean claw_open = false;

    public enum State{
        FREE,
        DOWN,
        GIVEHP,
        HOLDING_BLOCK
    }

    private SuperStructure(@NonNull HardwareMap hardwareMap) {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        shifter = hardwareMap.get(Servo.class, "shifter");
        clawUp = hardwareMap.get(Servo.class, "clawUp");
        armUp = hardwareMap.get(Servo.class, "armUp");
        wristUp = hardwareMap.get(Servo.class, "wristUp");
        slideUp = hardwareMap.get(Servo.class, "slideUp");

        slideFront = hardwareMap.get(DcMotorEx.class, "slideFront");
        claw = hardwareMap.get(Servo.class, "claw");
        spin = hardwareMap.get(Servo.class, "spin");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armSpin = hardwareMap.get(Servo.class, "armSpin");
        armWrist = hardwareMap.get(Servo.class, "armWrist");

        DcMotorEx.RunMode mode = DcMotorEx.RunMode.RUN_TO_POSITION;

        shifter.setPosition(ServoConstants.SHIFTER_NORMAL.value);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setTargetPosition(0);
        slideRight.setTargetPosition(0);
        slideLeft.setMode(mode);
        slideRight.setMode(mode);
        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        slideRight.setDirection(DcMotorEx.Direction.REVERSE);
        slideLeft.setPower(1);
        slideRight.setPower(1);

        slideFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        try {
            led = hardwareMap.get(Servo.class, "LED");
        } catch (Exception e) {
            led = null;
        }
    }

    public static void init(@NonNull HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new SuperStructure(hardwareMap);
        }
    }

    public static SuperStructure getInstance() {
        if (instance == null) {
            throw new IllegalStateException("SuperStructure未初始化，请先调用init(hardwareMap)");
        }
        return instance;
    }

//    public boolean isFinished(int tolerance) {
//        return Math.abs((slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2
//                - ((slideLeft.getTargetPosition() + slideRight.getTargetPosition()) / 2)) < tolerance;
//    }
//    public boolean isFinished(){
//        return isFinished(15);
//    }
}