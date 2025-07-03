package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final NewMecanumDrive drive;
    private final DoubleSupplier x;
    private final DoubleSupplier rotate;
    private final DoubleSupplier y;
    private final BooleanSupplier shouldReset;
    private final DoubleSupplier powerCoefficient;

    public TeleOpDriveCommand(
            NewMecanumDrive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotate,
            BooleanSupplier shouldReset,
            DoubleSupplier powerCoefficient) {
        this.drive = drive;
        this.x = x;
        this.rotate = rotate;
        this.y = y;
        this.shouldReset = shouldReset;
        this.powerCoefficient = powerCoefficient;
    }

    @Override
    public void execute() {
        if (shouldReset.getAsBoolean()){
            drive.resetHeading();
            drive.resetOdo();
        }
        drive.setFieldCentric(x.getAsDouble(),y.getAsDouble(),rotate.getAsDouble(), powerCoefficient.getAsDouble());
        drive.update();
        drive.updateOdo();
    }
}
