package org.firstinspires.ftc.teamcode.opmodes.autos;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;

public abstract class AutoOpModeEx extends CommandOpModeEx {
    @Override
    public abstract void initialize();

    @Override
    public abstract void onStart();

    @Override
    public abstract void run();
    @Override
    public void functionalButtons() {}
}