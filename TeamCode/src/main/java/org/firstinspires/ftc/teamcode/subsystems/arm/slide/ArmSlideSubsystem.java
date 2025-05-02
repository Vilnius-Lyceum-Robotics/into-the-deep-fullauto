package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig.DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;

import java.util.logging.Level;

@Config
public class ArmSlideSubsystem {
    private DcMotorEx extensionMotor0, extensionMotor1, extensionMotor2;
    private DcMotorEx extensionEncoder;

    private TouchSensor limitSwitch;

    private MotionProfile motionProfile;
    private double encoderPosition = 0;

    private double encoderOffset = 0;
    private boolean overridePowerState = false;
    private double overridePowerValue = 0;

    private double feedForwardGain = FEED_FORWARD_GAIN;
    private boolean limitSwitchPressed = true;
    private boolean prevLimitSwitchPressed = false;
    private boolean encoderReset = true;

    private OPERATION_MODE prevOperationMode = OPERATION_MODE.NORMAL;

    private ElapsedTime timer = new ElapsedTime();

    private PIDController holdPointPID = new PIDController(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);


    public ArmSlideSubsystem(HardwareMap hardwareMap) {
        ArmState.resetAll();

        extensionMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);

        limitSwitch = hardwareMap.get(TouchSensor.class, LIMIT_SW_NAME);

        extensionMotor0.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        extensionMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        extensionEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "SLIDE",
                MotionProfile.Type.ACCELERATION_LIMITED,
                ACCELERATION,
                DECELERATION,
                MAX_VELOCITY,
                CREEP,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);

        motionProfile.enableTelemetry(true);
        resetEncoder();
        timer.reset();
    }


    public void setTargetPosition(double position) {
        VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "NEW SLIDE EXTENSION OF " + position + " JUST SET");

        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        motionProfile.setTargetPosition(clamp(position, MIN_POSITION, MAX_POSITION));
    }


    public boolean reachedTargetPosition() {
        return reachedPosition(getTargetPosition());
    }

    public boolean reachedPosition(double position) {
        return Math.abs(getPosition() - position) <= ERROR_MARGIN;
    }

    public double getPosition() {
        return encoderPosition;
    }


    public double getExtension(){
        return mapToRange(getPosition(), MIN_POSITION, MAX_POSITION, 0, 1);
    }

    public double getTargetPosition() {
        return motionProfile.getTargetPosition();
    }


    public double getTargetExtension() {
        return mapToRange(getPosition(), MIN_POSITION, MAX_POSITION, 0, 1);
    }


    public void setHangCoefficients() {
        feedForwardGain = FEED_FORWARD_GAIN_HANG;
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
        feedForwardGain = FEED_FORWARD_GAIN;
        motionProfile.updateCoefficients(
                ACCELERATION,
                DECELERATION,
                MAX_VELOCITY,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
    }


    public void enablePowerOverride(double power){
        overridePowerState = true;
        overridePowerValue = power;
    }

    public void disablePowerOverride(){
        overridePowerState = false;
        overridePowerValue = 0;
    }

    public boolean getPowerOverride(){
        return overridePowerState;
    }

    private void setMotorPower(double power) {
        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }

    public boolean getLimitSwitchState(){
        return limitSwitchPressed;
    }

    public double getT(){
        return motionProfile.getT();
    }

    private void resetEncoder(){
        encoderOffset = extensionEncoder.getCurrentPosition();
    }

    public void updateCoefficientsForOperationMode(OPERATION_MODE operationMode){
        if (operationMode == OPERATION_MODE.HANG) {setHangCoefficients();}
        else if (operationMode == OPERATION_MODE.NORMAL) {setDefaultCoefficients();}
    }


    public void periodic(double armAngleDegrees, OPERATION_MODE operationMode) {
        limitSwitchPressed = limitSwitch.isPressed();
        encoderPosition = -(extensionEncoder.getCurrentPosition() - encoderOffset) / 8192d;

        if (DEBUG_MODE){
            setDefaultCoefficients();
            holdPointPID.setPID(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);
        }

        double feedForwardPower = Math.sin(Math.toRadians(armAngleDegrees)) * feedForwardGain;
        double power = motionProfile.getPower(getPosition()) + feedForwardPower;

        if (operationMode == OPERATION_MODE.HOLD_POINT && motionProfile.getTargetPosition() != 0) {
            power = holdPointPID.calculate(getPosition(), motionProfile.getTargetPosition()) + feedForwardPower;

            if (prevOperationMode != OPERATION_MODE.HOLD_POINT) {
                prevOperationMode = OPERATION_MODE.HOLD_POINT;
                VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "SLIDES HOLDING POINT");
            }
        }
        power = clamp(power, -0.9, 9);


        if (limitSwitchPressed && motionProfile.getTargetPosition() == 0) {
            extensionMotor1.setPower(0);
            extensionMotor2.setPower(0);

            if (!prevLimitSwitchPressed) {
                timer.reset();
            }

            else if (timer.seconds() < 0.4){
                extensionMotor0.setPower(-0.3);
            }

            else if (timer.seconds() > 0.4) {
                setMotorPower(0);

                if (timer.seconds() > 0.7 && !encoderReset) {
                    resetEncoder();
                    encoderReset = true;
                }
            }
        }

        else {
            encoderReset = false;

            if (overridePowerState) {
                extensionMotor0.setPower(clamp(overridePowerValue, -0.5, 0.5));
                extensionMotor1.setPower(0);
                extensionMotor2.setPower(0);
            }

            else if (reachedTargetPosition()) {
                extensionMotor0.setPower(0);
                extensionMotor1.setPower(power);
                extensionMotor2.setPower(power);
            }
            else {setMotorPower(power);}
        }

        prevLimitSwitchPressed = limitSwitchPressed;

        Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("ARM SLIDE SUBSYSTEM POWER TIMER: ", timer.seconds());
        telemetry.addData("ARM SLIDE SUBSYSTEM ENCODER RESET: ", encoderReset ? 1 : 0);
        telemetry.addData("ARM SLIDE SUBSYSTEM LIMIT SWITCH STATE: ", limitSwitchPressed);
        telemetry.addData("ARM SLIDE SUBSYSTEM PREV LIMIT SWITCH STATE: ", prevLimitSwitchPressed);
    }
}