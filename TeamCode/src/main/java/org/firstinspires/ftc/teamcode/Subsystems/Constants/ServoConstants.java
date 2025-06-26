package org.firstinspires.ftc.teamcode.Subsystems.Constants;
import com.qualcomm.robotcore.hardware.Servo;

public enum ServoConstants {
    CLAW_OPEN(0),
    CLAW_CLOSE(0.5),
    WRIST_PARALLEL(0.58),
    WRIST_DOWN(0.18),
    WRIST_HANDOVER(0.7),
    ARM_SPINNER_FRONT(0.45),
    ARM_SPINNER_LEFT45(0.6),
    ARM_SPINNER_RIGHT45(0.27),
    ARM_SPINNER_BACK(0.05),
    ARM_WRIST_PREINTAKE(0.17),
    ARM_WRIST_DOWN(0.11),
    ARM_WRIST_TURN(0.27),
    ARM_WRIST_HANDOVER(0.9),
    ARM_WRIST_FREE(0.5),

    UP_CLAW_OPEN(1),
    UP_CLAW_CLOSE(0.6),
    UP_SLIDE_MIN(0),
    UP_SLIDE_MAX(0.58),
    UP_WRIST_BASKET(0.55),
    UP_WRIST_PARALLEL(0.45),
    UP_WRIST_HANDOVER(0.9),
    UP_ARM_BASKET(0.7),
    UP_ARM_PARALLEL(0.28),
    UP_ARM_HANDOVER(0.07),
    ;



    public final double value;

    ServoConstants(double pos) {
        value = pos;
    }
    public void setToServo(Servo servo){
        servo.setPosition(this.value);
    }
}
