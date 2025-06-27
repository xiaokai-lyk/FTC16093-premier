package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {
    public abstract void onStart();
    public abstract void addButtons();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        addButtons();

        waitForStart();

        onStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}
