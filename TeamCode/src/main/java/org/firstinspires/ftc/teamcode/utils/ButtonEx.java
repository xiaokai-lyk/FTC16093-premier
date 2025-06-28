package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.button.Button;


import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class ButtonEx extends Button {
    private final BooleanSupplier booleanSupplier;

    public ButtonEx(BooleanSupplier supplier) {
        booleanSupplier = supplier;
    }

    @Override
    public boolean get() {
        return booleanSupplier.getAsBoolean();
    }
}