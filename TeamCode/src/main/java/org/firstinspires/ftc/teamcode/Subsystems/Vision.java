package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import edu.wpi.first.math.MathUtil;
import lombok.Setter;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Vision {
    private final Limelight3A camera;

    private final Servo led = null;

    public static double ledPWM = 0.5;

    public static double CAMERA_HEIGHT = 250;
    public static double CAMERA_ANGLE = -45.0;
    public static double TARGET_HEIGHT = 19.05;

    public static double sampleToRobotDistance = 293.6;

    @Setter private double colorVal = 0.0;

    Telemetry telemetry;

    public Vision(@NonNull final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");

//        led = hardwareMap.get(Servo.class, "LED");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Deprecated
    public void setLedPWM() {
        led.setPosition(ledPWM);
    }

    public void setColor(double color){
        /*0: red, 1:blue, 2:yellow*/
        colorVal = color;
    }
    public void initialize() {
        camera.setPollRateHz(50);
        camera.start();
    }

    public double getDistance(double ty) {
        if (MathUtil.isNear(0, ty, 0.01)) {
            return 0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - sampleToRobotDistance;
    }

    private Double getTurnServoDegree(@NonNull LLResult result_m){
        return result_m.getPythonOutput()[3];
    }

    public LLResult getResult() {
        return camera.getLatestResult();
    }

    public void update(boolean debugMode){
        LLResult result;
        camera.updatePythonInputs(new double[]{colorVal, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = getResult();
        if(debugMode && result != null){
            telemetry.addData("isValid", result.isValid());
            telemetry.addData("distance", getDistance(result.getTy()));
            telemetry.addData("getTurnServoDegree", getTurnServoDegree(result));
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("staleness",result.getStaleness());
        }
    }
    public void update() {
        update(true);
    }

}
