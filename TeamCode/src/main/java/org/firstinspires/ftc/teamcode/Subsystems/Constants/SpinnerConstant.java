package org.firstinspires.ftc.teamcode.Subsystems.Constants;


import com.qualcomm.robotcore.hardware.Servo;

public enum SpinnerConstant {
    DEG1(0),
    DEG2(0.2),
    DEG3(0.4),
    PARALLEL(0.53),
    DEG4(0.7),
    DEG5(1),


    GIVE_HP(0.8)
    ;
    public final double value;

    SpinnerConstant(double pos) {
        value = pos;
    }
    public void setToServo(Servo servo){
        servo.setPosition(this.value);
    }
}
