package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import lombok.Setter;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vision {
    private final Limelight3A camera;

    private final Servo led;

    @Setter private double colorVal = 0.0;
    /*0: red, 1:blue, 2:yellow*/

    Telemetry telemetry;

    public Vision(@NonNull final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        led = hardwareMap.get(Servo.class, "LED");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void setLed(boolean enable){
        if(enable)led.getController().pwmEnable();
        else led.getController().pwmDisable();
    }

    public void initialize() {
        led.setPosition(0.5);
        setLed(true);
        camera.setPollRateHz(50);
        camera.start();
    }

    public double getDistance(double rawDistanceMM) {
        double cameraToRobotOffset = -22;

        return Math.abs(rawDistanceMM) - cameraToRobotOffset;
    }

    private Double getTurnServoDegree(@NonNull LLResult result_m){
        return result_m.getPythonOutput()[3];
    }

    private double rawVerticalDistance(double ty){
        double CAMERA_HEIGHT = 250;
        double CAMERA_ANGLE = -45.0;
        double TARGET_HEIGHT = 19.05;

        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
    }

    private double getHorizontalDistance(double ty, double tx){
        double offset = 1;

        return rawVerticalDistance(ty) * Math.tan(Math.toRadians(tx));
    }

    private LLResult getResult() {
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
            telemetry.addData("horizontal distance",getHorizontalDistance(getDistance(result.getTy()), result.getTx()));
        }
    }
}
