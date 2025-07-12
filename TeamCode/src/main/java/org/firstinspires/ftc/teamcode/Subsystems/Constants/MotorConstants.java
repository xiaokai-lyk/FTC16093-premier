package org.firstinspires.ftc.teamcode.Subsystems.Constants;

public enum MotorConstants {
    FRONT_MAX(490),
    FRONT_NEAR(150),
    FRONT_TOLERANCE(10),
    LIFT_HIGH(955),
    LIFT_ABOVE_BASKET_TOLERANCE(700),
    LIFT_PARK_AIM(450),
    LIFT_HIGH_CHAMBER(490),


    FINAL_ASCENT_THRESHOLD(400),
    ;

    public final int value;

    MotorConstants(int pos) {
        value = pos;
    }
}