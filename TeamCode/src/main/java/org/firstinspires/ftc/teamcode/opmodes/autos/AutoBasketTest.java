package org.firstinspires.ftc.teamcode.opmodes.autos;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;


@Autonomous(name = "Auto Basket test", group = "Auto")
public class AutoBasketTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        LiftArm liftArm = new LiftArm(hardwareMap);
        AutoCommand auto = new AutoCommand(new FrontArm(hardwareMap), liftArm);
        auto.autoReleaseHigh().schedule();
        boolean a = true;
        while (opModeIsActive()){
            CommandScheduler.getInstance().run();
            if(getRuntime()>7 && a){
                a = false;
                new SequentialCommandGroup(
                        auto.autoReleaseHigh(),
                        new WaitUntilCommand(liftArm::isFinished),
                        auto.autoReleaseHigh(),
                        new WaitUntilCommand(liftArm::isFinished),
                        auto.autoReleaseHigh()
                ).schedule();
                telemetry.addLine("pressed!");
            }
            telemetry.addData("lift slide info", liftArm.slideInfo());
            telemetry.update();
        }
    }

}
