package org.firstinspires.ftc.teamcode.Subsystems.Constants;

public enum MotorConstants {
    FRONT_MAX(490),
    FRONT_NEAR(100),
    FRONT_TOLERANCE(10),
    LIFT_HIGH(980),
    LIFT_ABOVE_BASKET_TOLERANCE(700),
    LIFT_HIGH_CHAMBER(460),

    SHIFTER_TO_SLOW_THRESHOLD(600),
    MANUAL_ASCENT_THRESHOLD(900),
    ;

    public final int value;

    MotorConstants(int pos) {
        value = pos;
    }
}