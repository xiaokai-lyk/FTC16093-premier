package org.firstinspires.ftc.teamcode.Subsystems.Constants;
import com.qualcomm.robotcore.hardware.Servo;

public enum ServoConstants {
    CLAW_OPEN(0.6),
    CLAW_CLOSE(0.15),
    CLAW_HAS_BLOCK_MIN_DEGREE( 255),
    CLAW_CHECK(0.08),
    WRIST_PARALLEL(0.58),
    WRIST_DOWN(0.13),
    WRIST_HANDOVER(0.7),
    ARM_SPINNER_FRONT(0.45),
    ARM_SPINNER_LEFT45(0.6),
    ARM_SPINNER_RIGHT45(0.27),
    ARM_SPINNER_BACK(0.05),
    ARM_WRIST_PREINTAKE(0.19),
    ARM_WRIST_DOWN(0.1),
    ARM_WRIST_TURN(0.27),
    ARM_WRIST_HANDOVER(0.9),
    ARM_WRIST_FREE(0.5),

    UP_CLAW_OPEN(0.2),
    UP_CLAW_CLOSE(0.83),
    UP_SLIDE_MIN(0.59),
    UP_SLIDE_MAX(0.18),
    UP_WRIST_BASKET(0.55),
    UP_WRIST_PARALLEL(0.32),
    UP_WRIST_HANDOVER(0.9),
    UP_WRIST_WALL(0.65),
    UP_ARM_BASKET(0.7),
    UP_ARM_PARALLEL(0.25),
    UP_ARM_HANDOVER(0.07),
    UP_ARM_WALL(0.99),
    ;



    public final double value;

    ServoConstants(double pos) {
        value = pos;
    }
    public void setToServo(Servo servo){
        servo.setPosition(this.value);
    }
}