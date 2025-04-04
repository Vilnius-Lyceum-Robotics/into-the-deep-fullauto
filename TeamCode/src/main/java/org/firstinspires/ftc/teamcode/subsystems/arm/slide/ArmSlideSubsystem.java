package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

@Config
public class ArmSlideSubsystem extends VLRSubsystem<ArmSlideSubsystem> {
    private DcMotorSimple extensionMotor0, extensionMotor1, extensionMotor2;
    private DcMotorEx extensionEncoder;

    private TouchSensor limitSwitch;

    private MotionProfile motionProfile;

    private double encoderPosition = 0;
    private double lastPositionChangeTime = 0;

    private double encoderOffset = 0;

    private OperationMode operationMode = OperationMode.NORMAL;
    private boolean overridePower = false;
    private double feedForwardGain = FEED_FORWARD_GAIN;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        ArmState.resetAll();

        extensionMotor0 = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME_2);

        limitSwitch = hardwareMap.get(TouchSensor.class, LIMIT_SW_NAME);

        extensionMotor0.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

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
    }


    public void setTargetPosition(TargetPosition position) {
        setTargetPosition(position.extension);
    }

    public void setTargetPosition(double position) {
        //System.out.print("Setting target to");
        //System.out.println(position);
        lastPositionChangeTime = System.currentTimeMillis();
        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        motionProfile.setTargetPosition(clamp(position, MIN_POSITION, MAX_POSITION));
    }


    // This method should only be used for commands.
    // Returns true if position has been reached, or position timeout has occurred.
    // This is so the whole command system doesn't freeze if the slides are unable to reach the specified position
    public boolean reachedTargetPosition() {
        if (lastPositionChangeTime != 0 && System.currentTimeMillis() - lastPositionChangeTime > ERROR_TIMEOUT_MILLIS) {
            return true;
        }
        return reachedPosition(getTargetPosition());
    }

    public boolean reachedTargetPositionNoOverride() {
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


    public void incrementTargetPosition(double increment) {
        //System.out.printf("sniegas " + clamp(getTargetPosition() + increment, MIN_MANUAL_ADJUST_POSITION, HORIZONTAL_EXTENSION_LIMIT) + "\n");
        motionProfile.setTargetPosition(clamp(getTargetPosition() + increment, MIN_MANUAL_ADJUST_POSITION, HORIZONTAL_EXTENSION_LIMIT));
    }


    public void setHangCoefficientsFast() {
        feedForwardGain = FEED_FORWARD_GAIN_HANG;
        motionProfile.updateCoefficients(
                ACCELERATION_HANG_FAST,
                DECELERATION_HANG_FAST,
                MAX_VELOCITY_HANG_FAST,
                FEEDBACK_PROPORTIONAL_GAIN_HANG_FAST,
                FEEDBACK_INTEGRAL_GAIN_HANG_FAST,
                FEEDBACK_DERIVATIVE_GAIN_HANG,
                VELOCITY_GAIN_HANG_FAST,
                ACCELERATION_GAIN_HANG);
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



    public void setPowerOverride(boolean condition){
        overridePower = condition;
    }


    public boolean getPowerOverride(){
        return overridePower;
    }


    public void setMotorPower(double power) {
        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }


    public void checkLimitSwitch() {
        if (limitSwitch.isPressed()) {
            //System.out.println("LIMIT ON");
            //extensionEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderOffset = extensionEncoder.getCurrentPosition();
//            extensionEncoder.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            extensionEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public boolean getLimitSwitchState(){
        return limitSwitch.isPressed();
    }


    public void setOperationMode(OperationMode operationMode){
        this.operationMode = operationMode;
    }


    public OperationMode getOperationMode(){
        return operationMode;
    }


    public void periodic(double armAngleDegrees) {
        checkLimitSwitch();

        encoderPosition = -(extensionEncoder.getCurrentPosition() - encoderOffset) / 8192d;

        double feedForwardPower = Math.sin(Math.toRadians(armAngleDegrees)) * feedForwardGain;
        double power = motionProfile.getPower(getPosition()) + feedForwardPower;
        power = clamp(power, -1, 1);


        if (!overridePower) {
            if (operationMode == OperationMode.NORMAL) {
                setDefaultCoefficients();

                if (reachedTargetPositionNoOverride()) {
                    extensionMotor0.setPower(0);

                    if (getTargetExtension() == TargetPosition.RETRACTED.extension) {
                        extensionMotor1.setPower(0);
                        extensionMotor2.setPower(0);

                    } else{
                        extensionMotor1.setPower(power);
                        extensionMotor2.setPower(power);
                    }

                } else setMotorPower(power);
            } else{
                setMotorPower(power);
            }
        }
    }
}