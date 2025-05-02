package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig.DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.*;
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
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem.mapToRange;



import java.util.logging.Level;


public class ArmRotatorSubsystem {
    private DcMotorSimple motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;

    private double encoderPosition = 0;
    private double encoderOffset = 0;

    private PIDController holdPointPID = new PIDController(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);
    private ElapsedTime timer = new ElapsedTime();
    private boolean encoderReset = false;

    private RevTouchSensor breamBreak;
    private boolean prevBreamBreakState = false;

    private OPERATION_MODE operationMode = OPERATION_MODE.NORMAL;
    private OPERATION_MODE prevOperationMode = OPERATION_MODE.NORMAL;

    private boolean powerOverride_state = false;
    private double powerOverride_value = 0;


    public ArmRotatorSubsystem (HardwareMap hardwareMap) {
        ArmState.resetAll();

        motor = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        motionProfile.setTargetPosition(0);
        timer.reset();
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
            setDefaultCoefficients();
        }
        motionProfile.setTargetPosition(clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE));
    }


    public double getAngleDegrees() {
        return encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(motionProfile.getTargetPosition());
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
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
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


    public double getT(){
        return motionProfile.getT();
    }

    public void updateCoefficientsForOperationMode(OPERATION_MODE operationMode){
        this.operationMode = operationMode;
        if (operationMode == OPERATION_MODE.HANG) {setHangCoefficients();}
        else if (operationMode == OPERATION_MODE.NORMAL) {setDefaultCoefficients();}
    }


    private void resetEncoder(){
        encoderOffset = thoughBoreEncoder.getCurrentPosition();
    }

    public void enablePowerOverride(double power){
        powerOverride_state = true;
        powerOverride_value = power;
    }

    public void disablePowerOverride(){
        powerOverride_value = 0;
        powerOverride_state = false;
    }


    public void periodic(double slideExtension, OPERATION_MODE operationMode) {
        encoderPosition = (thoughBoreEncoder.getCurrentPosition() - encoderOffset);
        double currentAngle = getAngleDegrees();
        boolean currentBeamBreakState = breamBreak.isPressed();

        if (DEBUG_MODE){
            setDefaultCoefficients();
            holdPointPID.setPID(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);
        }

        if (powerOverride_state){
            motor.setPower(powerOverride_value);
            return;
        }

        double feedForward = mapToRange(slideExtension, 0, 1, FEEDFORWARD_GAIN, EXTENDED_FEEDFORWARD_GAIN);
        double feedForwardPower = Math.cos(Math.toRadians(currentAngle)) * feedForward;
        double power = motionProfile.getPower(currentAngle) + feedForwardPower;

        if (operationMode == OPERATION_MODE.HOLD_POINT && motionProfile.getTargetPosition() != 0){
            power = holdPointPID.calculate(currentAngle, motionProfile.getTargetPosition()) + feedForwardPower;

            if (prevOperationMode != OPERATION_MODE.HOLD_POINT){
                prevOperationMode = OPERATION_MODE.HOLD_POINT;
                VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "ROTATOR HOLDING POINT");
            }
        }

        power = clamp(power, -1, 1);


        if (currentBeamBreakState && motionProfile.getTargetPosition() == 0) {
            if (!prevBreamBreakState) {timer.reset();}
            else if(timer.seconds() < 1) {
                power = -0.07;

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
        telemetry.addData("ARM ROTATOR SUBSYSTEM BEAM BREAK", breamBreak.getValue());

        motor.setPower(power);
    }
}