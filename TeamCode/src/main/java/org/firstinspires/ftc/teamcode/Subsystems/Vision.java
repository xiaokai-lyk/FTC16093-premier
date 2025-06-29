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


    @Setter
    private SampleColor detectionColor = SampleColor.BLUE;
    private LLResult result;

    public static double CAMERA_HEIGHT = 307.0 - 16;
    public static double CAMERA_ANGLE = -45.0;
    public static double TARGET_HEIGHT = 19.05;

    public static double strafeConversionFactor = 6.6667;
    public static double cameraStrafeToBot = -20;

    public static double sampleToRobotDistance = 145;

    Telemetry telemetry;

    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        try {
            led = hardwareMap.get(Servo.class, "LED");
        } catch (Exception e) {
            led = null;
        }
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void setLedPWM() {
        if(led!=null)led.setPosition(ledPWM);
    }

    public void initializeCamera() {
        camera.setPollRateHz(50);
        camera.start();
    }


    public enum SampleColor {
        RED(0.0),
        BLUE(1.0),
        YELLOW(2.0);

        private final double colorVal;

        SampleColor(double v) {
            this.colorVal = v;
        }
    }


    public boolean isTargetVisible() {
        if (result == null) {
            return false;
        }
        return !MathUtil.isNear(0, result.getTa(), 0.0001);
    }

    public double getDistance() {
        double ty = result.getTy();
        if (MathUtil.isNear(0, ty, 0.01)) {
            return 0;
        }
        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
        return Math.abs(distanceMM) - sampleToRobotDistance;
    }

    // Get the strafe
    public double getStrafeOffset() {
        double tx = result.getTx();
        if (tx != 0) {
            return tx * strafeConversionFactor - cameraStrafeToBot;
        }
        return 0;
    }

    private Double getTurnServoDegree(){
        return result.getPythonOutput()[3];
    }

    public void periodic() {
        camera.updatePythonInputs(
                new double[]{detectionColor.colorVal, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = camera.getLatestResult();

        if (result != null) {
            telemetry.addData("Strafe Offset", getStrafeOffset());
            telemetry.addData("Distance", getDistance());
            telemetry.addData("Turn Servo Degrees", getTurnServoDegree());

            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.update();
        }
    }
}
