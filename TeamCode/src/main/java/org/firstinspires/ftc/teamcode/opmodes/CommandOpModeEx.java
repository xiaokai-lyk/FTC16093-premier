package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;

public abstract class CommandOpModeEx extends CommandOpMode {
    public abstract void onStart();
    public abstract void functionalButtons();
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        functionalButtons();

        waitForStart();

        onStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }
}
