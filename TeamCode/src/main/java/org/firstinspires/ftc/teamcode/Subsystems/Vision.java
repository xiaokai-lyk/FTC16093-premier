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
import org.firstinspires.ftc.teamcode.Subsystems.Constants.MotorConstants;

public class Vision {
    private final Limelight3A camera;
    private final Servo led;

    @Setter private double colorVal = 0.0;
    /*0: red, 1:blue, 2:yellow*/

    Telemetry telemetry;

    public Vision(@NonNull final HardwareMap hardwareMap, Telemetry telemetry) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        led = hardwareMap.get(Servo.class, "LED");
        if(telemetry != null)this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        else this.telemetry = FtcDashboard.getInstance().getTelemetry();
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

    private Double getTurnServoDegree(@NonNull LLResult result_m){
        return result_m.getPythonOutput()[3];
    }

    private double getDistanceMM(double ty){
        double CAMERA_HEIGHT = 250;
        double CAMERA_ANGLE = -45.0;
        double TARGET_HEIGHT = 19.05;
        double offset = 22;

        double angleToGoalDegrees = CAMERA_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians) + offset;
    }

    public double getArmSpinnerPos(@NonNull LLResult result) {
        double tx = result.getTx();
        double pos = -0.00000213149 * Math.pow(tx, 4) + 0.0000395987 * Math.pow(tx, 3)
                + -0.0000261975 * Math.pow(tx, 2) + 0.010841 * tx + 0.213606;
        return Math.min(Math.max(pos, 0.03), 0.98);
    }

    private double servoPos2Angle(double servoPos){
        return -218.01505 * servoPos + 84.32738;
    }

    public int getSlideTarget(@NonNull LLResult result) {
        double armLengthMM = 146.121;
        double servoPos = getArmSpinnerPos(result);
        double servoAngle = servoPos2Angle(servoPos);
        double armVerticalLength = Math.cos(Math.toRadians(servoAngle)) * armLengthMM;
        double slideTargetDistanceMM = getDistanceMM(result.getTy()) - armVerticalLength;

        int target = (int) (1.39244 * slideTargetDistanceMM - 2.35489);

        return Math.min(Math.max(0, target), MotorConstants.FRONT_FAR.value);
    }

    public LLResult getResult() {
        return camera.getLatestResult();
    }

    public boolean resultValid(@NonNull LLResult result){
        return result.getTa() != 0 && result.getStaleness() < 30;
    }

    public void update(boolean debugMode){
        LLResult result;
        camera.updatePythonInputs(new double[]{colorVal, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        result = getResult();
        if(debugMode && result != null){
            telemetry.addData("isValid", resultValid(result));
            telemetry.addData("distance (mm)", (getDistanceMM(result.getTy())));
            telemetry.addData("getTurnServoDegree", getTurnServoDegree(result));
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("staleness",result.getStaleness());
            telemetry.addData("spinner pos", getArmSpinnerPos(result));
            telemetry.addData("slide target", getSlideTarget(result));
        }
    }
}
