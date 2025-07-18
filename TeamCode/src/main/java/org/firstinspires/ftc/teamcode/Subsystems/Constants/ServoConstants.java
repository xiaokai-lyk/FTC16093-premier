package org.firstinspires.ftc.teamcode.Subsystems.Constants;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

public enum ServoConstants {
    CLAW_OPEN(0.85),
    CLAW_CLOSE(0.51),
    CLAW_HAS_BLOCK_MIN_DEGREE( 220),
    CLAW_CHECK(0.35),
    WRIST_PARALLEL(0.58),
    WRIST_DOWN(0.16),
    WRIST_HANDOVER(0.9),
    // range of arm spin: [0.02, 0.98]
    // if you set a position that exceed this servo would become continuous!
    ARM_SPINNER_FRONT(0.38),
    ARM_SPINNER_LEFT45(0.6),
    ARM_SPINNER_RIGHT45(0.27),
    ARM_SPINNER_BACK(0.98),
    ARM_WRIST_PREINTAKE(0.35),
    ARM_WRIST_DOWN(0.30),
    ARM_WRIST_TURN(0.45),
    ARM_WRIST_HANDOVER(0.72),
    ARM_WRIST_FREE(0.70),
    ARM_WRIST_CHAMBER_INTAKE(0.55),

    UP_CLAW_OPEN(0.3),
    UP_CLAW_CLOSE_CAN_SLIDE(0.95),
    UP_CLAW_CLOSE(0.97),
    UP_WRIST_BASKET(0.6),
    UP_WRIST_PARALLEL(0.7),
    UP_WRIST_HANDOVER(0.3),
    UP_WRIST_WALL(0.45),
    UP_ARM_BASKET(0.36),
    UP_ARM_PARALLEL(0.68),
    UP_ARM_HANDOVER(0.8),
    UP_ARM_WALL(0.22),
    UP_ARM_UPWARD(0.5),
    UP_ARM_BACK(0.26),

    SHIFTER_NORMAL(0.6),
    SHIFTER_SLOW(1),

    ASCENT_LEFT_UP(0.32),
    ASCENT_LEFT_MID(0.65),
    ASCENT_LEFT_DOWN(0.75),
    ASCENT_RIGHT_UP(0.68),
    ASCENT_RIGHT_MID(0.39),
    ASCENT_RIGHT_DOWN(0.26),
    ;



    public final double value;

    ServoConstants(double pos) {
        value = pos;
    }
    public void setToServo(@NonNull Servo servo){
        servo.setPosition(this.value);
    }
}