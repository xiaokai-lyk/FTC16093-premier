package org.firstinspires.ftc.teamcode.Subsystems.Constants;

public enum MotorConstants {
    FRONT_MAX(490),
    FRONT_NEAR(100),
    FRONT_TOLERANCE(10),
    LIFT_HIGH(980),
    LIFT_ABOVE_BASKET_TOLERANCE(700),
    AUTO_LIFT_ABOVE_BASKET_TOLERANCE(100),
    LIFT_HIGH_CHAMBER(460),

    SLOW_ASCENT_THRESHOLD(700),
    ;

    public final int value;

    MotorConstants(int pos) {
        value = pos;
    }
}