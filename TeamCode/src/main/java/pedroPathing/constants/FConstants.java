package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 12.57;

        FollowerConstants.xMovement = 93.677;
        FollowerConstants.yMovement = 73.163;

        FollowerConstants.forwardZeroPowerAcceleration = -30.265;
        FollowerConstants.lateralZeroPowerAcceleration = -62.925;

        /*FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.8,0,0.07,0.1);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.2,0,0.025,0.1);*/

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5, 0, 0.1, 0.1);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.15, 0, 0.03, 0.1);

        /*FollowerConstants.headingPIDFCoefficients.setCoefficients(1.5,0,0,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(3,0,0.2,0);*/

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.0, 0, 0.1, 0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2.5, 0, 0.15, 0);

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02,0,0.0003,0.3,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0.0008,0,0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;

        FollowerConstants.centripetalScaling = 0.00045;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.automaticHoldEnd = true;
        FollowerConstants.useVoltageCompensationInAuto = true;
    }
}