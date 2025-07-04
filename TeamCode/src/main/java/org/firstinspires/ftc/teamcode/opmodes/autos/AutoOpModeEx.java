package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;

public abstract class AutoOpModeEx extends CommandOpModeEx {
    @Override
    public abstract void initialize();

    @Override
    public void onStart() {
        CommandScheduler.getInstance().schedule(getAutonomousCommand());
    };

    @Override
    public abstract void run();
    @Override
    public void functionalButtons() {}

    public abstract Command getAutonomousCommand();
}