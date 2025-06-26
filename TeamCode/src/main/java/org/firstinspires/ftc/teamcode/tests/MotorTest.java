package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "motor test", group = "tests")
public class MotorTest extends LinearOpMode {
    public static boolean read_only = false;
    public static boolean reverse = false;

    public static String motor_name = "slideLeft";
    public static boolean power_mode = false;
    public static int pos = 0;
    public static double power = 0;
    public static boolean reset = true;
    private DcMotorEx motor = null;
    private final Telemetry telemetry_M =
            new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, motor_name);

        if (reverse) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        if(reset){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForStart();

        while (opModeIsActive()) {
            if(!read_only){
                if(power_mode){
                    motor.setPower(power);
                }else{
                    motor.setPower(power);
                    motor.setTargetPosition(pos);
                }
                motor.setMode(power_mode?DcMotor.RunMode.RUN_WITHOUT_ENCODER:DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                motor.setPower(0);
            }
            telemetry_M.addData("pos", motor.getCurrentPosition());
            telemetry_M.addData("target", motor.getTargetPosition());
            telemetry_M.addData("error", Math.abs(motor.getTargetPosition()-motor.getCurrentPosition()));
            telemetry_M.addData("mode", motor.getMode());
            telemetry_M.addData("power", motor.getPower());
            telemetry_M.update();
        }
    }
}
