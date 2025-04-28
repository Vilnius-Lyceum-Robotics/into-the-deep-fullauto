package org.firstinspires.ftc.teamcode.pedro.constants;

import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.MOTOR_LEFT_BACK;
import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.MOTOR_LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.MOTOR_RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration.MOTOR_RIGHT_FRONT;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    public static void initialize() {
        // Do nothing - just trigger the static block
    }

    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = MOTOR_LEFT_FRONT;
        FollowerConstants.leftRearMotorName = MOTOR_LEFT_BACK;
        FollowerConstants.rightFrontMotorName = MOTOR_RIGHT_FRONT;
        FollowerConstants.rightRearMotorName = MOTOR_RIGHT_BACK;

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        //FOR TUNING
//        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
//        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
//        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
//        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 10;

        FollowerConstants.xMovement = 84;
        FollowerConstants.yMovement = 68;

        FollowerConstants.forwardZeroPowerAcceleration = -37;
        FollowerConstants.lateralZeroPowerAcceleration = -70;

        FollowerConstants.turnHeadingErrorThreshold = 0.015; // default is 0.01

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.12,0,0.016,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2.5,0,0.13,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.007,0,0.00015,0.6,0.00004);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.25;
        FollowerConstants.centripetalScaling = 0.00045;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
//        //FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0,0.008,0); // original
//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5, 0, 0.063, 0); // secondary original
//        FollowerConstants.useSecondaryTranslationalPID = false;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0); // Not being used, @see useSecondaryTranslationalPID
//
//        //FollowerConstants.headingPIDFCoefficients.setCoefficients(0.45, 0, 0.0015, 0); // original
//        FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0); // secondary original
//        FollowerConstants.useSecondaryHeadingPID = false;
//        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0); // Not being used, @see useSecondaryHeadingPID
//
//        //FollowerConstants.drivePIDFCoefficients.setCoefficients(0.004,0,0.0000012,0.6,0); // original
//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01/1.9, 0, 0.000015/3, 0, 0); // secondary original
//        FollowerConstants.driveKalmanFilterParameters.dataCovariance = 100;
//        FollowerConstants.driveKalmanFilterParameters.modelCovariance = 60;
//        FollowerConstants.useSecondaryDrivePID = false;
//        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0); // Not being used, @see useSecondaryDrivePID
//
//        FollowerConstants.zeroPowerAccelerationMultiplier = 3;
//        FollowerConstants.centripetalScaling = 0.00025;
//
//        FollowerConstants.pathEndTimeoutConstraint = 500;
//        FollowerConstants.pathEndTValueConstraint = 0.995;
//        FollowerConstants.pathEndVelocityConstraint = 0.1;
//        FollowerConstants.pathEndTranslationalConstraint = 0.1;
//        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.nominalVoltage = 12.5;
    }
}
