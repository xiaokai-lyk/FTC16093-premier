package org.firstinspires.ftc.teamcode.opmodes.autos;

import org.firstinspires.ftc.teamcode.opmodes.CommandOpModeEx;

/**
 * 自动阶段基类，生命周期与手动阶段一致，但内容专为自动阶段设计。
 */
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