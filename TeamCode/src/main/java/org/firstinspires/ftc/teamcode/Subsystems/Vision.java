package org.firstinspires.ftc.teamcode.Subsystems;

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

    private Servo led;

    public static double ledPWM = 0.5;

    private LLResult result;

    public static double CAMERA_HEIGHT = 250;
    public static double CAMERA_ANGLE = -45.0;
    public static double TARGET_HEIGHT = 19.05;

    public static double sampleToRobotDistance = 145;

    Telemetry telemetry;

    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");

        led = hardwareMap.get(Servo.class, "LED");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void setLedPWM() {
        led.setPosition(ledPWM);
    }

    public void initializeCamera() {
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

    private Double getTurnServoDegree(){
        return result.getPythonOutput()[3];
    }


    public void update() {
        camera.updatePythonInputs(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = camera.getLatestResult();
        telemetry.addData("isValid", result.isValid());
        telemetry.clearAll();
        if(result.isValid()){
            telemetry.addData("distance", getDistance(result.getTy()));
        }
    }

}
