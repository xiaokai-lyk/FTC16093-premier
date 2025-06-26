package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.button.Button;


import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class ButtonEx extends Button {
    private final BooleanSupplier booleanSupplier;
    private static final HashSet<BooleanSupplier> usedSuppliers = new HashSet<>();

    public ButtonEx(BooleanSupplier supplier) {
        booleanSupplier = supplier;
        if(!usedSuppliers.add(supplier)){
            throw new RuntimeException(String.format("Trying to declare two identical buttons(%s).", supplier.toString()));
        }

    }

    @Override
    public boolean get() {
        return booleanSupplier.getAsBoolean();
    }
}