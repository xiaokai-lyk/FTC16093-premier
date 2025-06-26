package org.firstinspires.ftc.teamcode.Subsystems.Constants;


import com.qualcomm.robotcore.hardware.Servo;

public enum SpinnerConstant {
    PARALLEL(0.53),
    DEG1(0.6),
    DEG2(0.7),
    DEG3(0.8),
    DEG4(0.9),
    DEG5(1),
    ;
    public final double value;

    SpinnerConstant(double pos) {
        value = pos;
    }
    public void setToServo(Servo servo){
        servo.setPosition(this.value);
    }
}
