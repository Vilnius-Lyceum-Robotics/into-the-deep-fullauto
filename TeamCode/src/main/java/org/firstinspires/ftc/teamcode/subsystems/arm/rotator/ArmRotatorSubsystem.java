package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig.DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ACCELERATION_JERK_SLOW;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.BEAM_BREAK_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.DECELERATION_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.DECELERATION_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.DECELERATION_JERK_SLOW;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ENCODER_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ENCODER_TICKS_PER_ROTATION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ERROR_MARGIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_ACCELERATION_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_ACCELERATION_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_DECELERATION_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_FEEDBACK_DERIVATIVE_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_FEEDBACK_INTEGRAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_FEEDBACK_PROPORTIONAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_FEEDFORWARD_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.EXTENDED_VELOCITY_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_DERIVATIVE_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_DERIVATIVE_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_INTEGRAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_INTEGRAL_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_INTEGRAL_GAIN_HOLD_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_PROPORTIONAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_PROPORTIONAL_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDFORWARD_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.FEEDFORWARD_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MAX_VELOCITY_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MAX_VELOCITY_SLOW;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.VELOCITY_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.VELOCITY_GAIN_HANG;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;

import java.util.logging.Level;


public class ArmRotatorSubsystem {
    private DcMotorSimple motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;

    private static volatile double encoderPosition;
    private static volatile double encoderOffset;

    private PIDController holdPointPID = new PIDController(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);
    private ElapsedTime timer = new ElapsedTime();
    private boolean encoderReset = false;

    private RevTouchSensor breamBreak;
    private boolean prevBreamBreakState = false;

    private OPERATION_MODE operationMode = OPERATION_MODE.NORMAL;
    private OPERATION_MODE prevOperationMode = OPERATION_MODE.NORMAL;

    private boolean powerOverride_state = false;
    private double powerOverride_value = 0;

    private double powerLimit = 1;


    public ArmRotatorSubsystem (HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        breamBreak = hardwareMap.get(RevTouchSensor.class, BEAM_BREAK_NAME);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "ARM",
                MotionProfile.Type.JERK_LIMITED,
                ACCELERATION_JERK,
                DECELERATION_JERK,
                MAX_VELOCITY,
                ERROR_MARGIN,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);

        motionProfile.enableTelemetry(true);
        timer.reset();

        if  (ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)){
            thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            updateEncoderPosition();
            motionProfile.setTargetPosition(getAngleDegrees());
        }

        else {
            encoderOffset = 0;
            thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motionProfile.setTargetPosition(0);
            updateEncoderPosition();
        }


    }


    public void setTargetPosition(double angleDegrees, double slideExtension) {
        VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "NEW ROTATOR ANGLE OF " + angleDegrees + " JUST SET");

        if (operationMode == OPERATION_MODE.NORMAL && angleDegrees != 0) {
            double p = mapToRange(slideExtension, 0, 1, FEEDBACK_PROPORTIONAL_GAIN, EXTENDED_FEEDBACK_PROPORTIONAL_GAIN);
            double i = mapToRange(slideExtension, 0, 1, FEEDBACK_INTEGRAL_GAIN, EXTENDED_FEEDBACK_INTEGRAL_GAIN);
            double d = mapToRange(slideExtension, 0, 1, FEEDBACK_DERIVATIVE_GAIN, EXTENDED_FEEDBACK_DERIVATIVE_GAIN);
            double v = mapToRange(slideExtension, 0, 1, VELOCITY_GAIN, EXTENDED_VELOCITY_GAIN);
            double a = mapToRange(slideExtension, 0, 1, ACCELERATION_GAIN, EXTENDED_ACCELERATION_GAIN);
            double acceleration = mapToRange(slideExtension, 0, 1, ACCELERATION_JERK, EXTENDED_ACCELERATION_JERK);
            double deceleration = mapToRange(slideExtension, 0, 1, DECELERATION_JERK, EXTENDED_DECELERATION_JERK);
            double maxVelocity = mapToRange(slideExtension, 0, 1, MAX_VELOCITY, EXTENDED_MAX_VELOCITY);

            motionProfile.updateCoefficients(acceleration, deceleration, maxVelocity, p, i, d, v, a);
        }
        else{
            updateCoefficientsForOperationMode();
        }
        motionProfile.setTargetPosition(clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE));
    }


    public double getAngleDegrees() {
        return encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(motionProfile.getTargetPosition());
    }

    public double getTargetPosition(){
        return motionProfile.getTargetPosition();
    }


    public boolean reachedPosition(double angleDegrees) {
        return Math.abs(getAngleDegrees() - angleDegrees) < ERROR_MARGIN;
    }

    public void setHangCoefficients() {
        motionProfile.updateCoefficients(
                ACCELERATION_HANG,
                DECELERATION_HANG,
                MAX_VELOCITY_HANG,
                FEEDBACK_PROPORTIONAL_GAIN_HANG,
                FEEDBACK_INTEGRAL_GAIN_HANG,
                FEEDBACK_DERIVATIVE_GAIN_HANG,
                VELOCITY_GAIN_HANG,
                ACCELERATION_GAIN_HANG);
    }


    public void setDefaultCoefficients() {
        motionProfile.updateCoefficients(
                ACCELERATION_JERK,
                DECELERATION_JERK,
                MAX_VELOCITY,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
    }

    public void setSlowCoefficients() {
        motionProfile.updateCoefficients(
                ACCELERATION_JERK_SLOW,
                DECELERATION_JERK_SLOW,
                MAX_VELOCITY_SLOW,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
    }


    public double getT(){
        return motionProfile.getT();
    }

    public void updateCoefficientsForOperationMode(){
        if (operationMode == OPERATION_MODE.HANG) {
            setHangCoefficients();
        }
        else if (operationMode == OPERATION_MODE.NORMAL || operationMode == OPERATION_MODE.MAX_POWER_PULL) {
            setDefaultCoefficients();
        }
        else if (operationMode == OPERATION_MODE.NORMAL_SLOWER) {
            setSlowCoefficients();
        }
    }


    private void resetEncoder(){
        encoderOffset = thoughBoreEncoder.getCurrentPosition();
    }

    public void enablePowerOverride(double power){
        System.out.println("ROTATOR POWER OVERRIDE ENABLED");

        powerOverride_state = true;
        powerOverride_value = power;
    }

    public void disablePowerOverride(){
        powerOverride_value = 0;
        powerOverride_state = false;

        System.out.println("ROTATOR POWER OVERRIDE DISABLED");
    }


    private void updateEncoderPosition(){
        encoderPosition = (thoughBoreEncoder.getCurrentPosition() - encoderOffset);
    }

    public void setPowerLimit(double powerLimit){
        this.powerLimit = powerLimit;
    }


    public void periodic(double slideExtension, OPERATION_MODE operationMode) {
        updateEncoderPosition();
        double currentAngle = getAngleDegrees();
        boolean currentBeamBreakState = breamBreak.isPressed();
        this.operationMode = operationMode;

        if (operationMode == OPERATION_MODE.HOLD_POINT && prevOperationMode != OPERATION_MODE.HOLD_POINT){
            VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "ROTATOR HOLDING POINT");
        }
        if (prevOperationMode != operationMode) {prevOperationMode = operationMode;}


        if (DEBUG_MODE){
            holdPointPID.setPID(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);
            updateCoefficientsForOperationMode();
        }

        double feedForward = mapToRange(slideExtension, 0, 1, FEEDFORWARD_GAIN, EXTENDED_FEEDFORWARD_GAIN);
        if(operationMode == OPERATION_MODE.HANG){
            feedForward = FEEDFORWARD_GAIN_HANG;
        }

        double feedForwardPower = Math.cos(Math.toRadians(currentAngle)) * feedForward;
        double power = motionProfile.getPower(currentAngle) + feedForwardPower;

        if (operationMode == OPERATION_MODE.HOLD_POINT && motionProfile.getTargetPosition() != 0){
            power = holdPointPID.calculate(currentAngle, motionProfile.getTargetPosition()) + feedForwardPower;
        }

        power = clamp(power, -powerLimit, powerLimit);


        if (currentBeamBreakState && motionProfile.getTargetPosition() == 0) {
            if (!prevBreamBreakState) {timer.reset();}
            else if(timer.seconds() < 1) {
                power = -0.08;

                if (timer.seconds() > 0.6 && !encoderReset) {
                    resetEncoder();
                    encoderReset = true;
                }
            }
            else {power = 0;}
        }
        else {
            encoderReset = false;
            if (motionProfile.getTargetPosition() == 0 && currentAngle <= 10){
                power = -0.08;
            }
        }

        prevBreamBreakState = currentBeamBreakState;

        Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("ROTATOR POWER: ", power);

        if (powerOverride_state) {motor.setPower(powerOverride_value);}
        else {motor.setPower(power);}
    }
}