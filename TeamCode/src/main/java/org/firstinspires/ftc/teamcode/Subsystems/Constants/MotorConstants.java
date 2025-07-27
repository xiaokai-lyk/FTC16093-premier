package org.firstinspires.ftc.teamcode.Subsystems.Constants;

public enum MotorConstants {
    FRONT_FAR(490),
    FRONT_NEAR(150),
    FRONT_AUTO(300),
    FRONT_TOLERANCE(10),
    FRONT_FINISH_THRESHOLD(150),
    LIFT_HIGH(965),
    LIFT_LOW(410),
    LIFT_ABOVE_BASKET_TOLERANCE(700),
    LIFT_PARK_AIM(365),
    LIFT_PARK(165),
    LIFT_HIGH_CHAMBER(455),
    LIFT_HIGH_CHAMBER_FIRST(440),


    FINAL_ASCENT_THRESHOLD(400),
//    FINAL_ASCENT_SLIDE_FINISH(750)
    ;

    public final int value;

    MotorConstants(int value) {
        this.value = value;
    }
}