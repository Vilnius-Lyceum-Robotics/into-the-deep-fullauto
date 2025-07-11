package org.firstinspires.ftc.teamcode.subsystems.claw;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


public class ClawSubsystem extends VLRSubsystem<ClawSubsystem> implements ClawConfiguration {
    private Servo angleServo, twistServo, grabServos;
    private RevTouchSensor proximitySensor;
    private final LowPassFilter proximityFilter = new LowPassFilter(0.89);

    private VerticalRotation targetAngle = VerticalRotation.UP;
    private GripperState clawState = GripperState.CLOSED;

    public GripperState getClawState() {
        return clawState;
    }

    protected void initialize(HardwareMap hardwareMap) {
        angleServo = hardwareMap.get(Servo.class, ANGLE_SERVO);
        twistServo = hardwareMap.get(Servo.class, TWIST_SERVO);
        grabServos = hardwareMap.get(Servo.class, GRAB_SERVO);

        proximitySensor = hardwareMap.get(RevTouchSensor.class, PROXIMITY_SENSOR);
    }


    public void setTargetAngle(VerticalRotation rotation) {
        this.targetAngle = rotation;
        angleServo.setPosition(rotation.pos);
    }

    public void setTargetAngle(double pos) {
        angleServo.setPosition(pos);
    }

    public VerticalRotation getTargetAngle() {
        return targetAngle;
    }

    public void setHorizontalRotation(HorizontalRotation rotation) {
        twistServo.setPosition(rotation.pos);
    }

    public void setHorizontalRotation(double rotationPos) {
        twistServo.setPosition(clamp(rotationPos, HORIZONTAL_ROTATION_MIN, HORIZONTAL_ROTATION_MAX));
    }

//    public void setHorizontalRotation(double rotationPos) {
//        twistServo.setPosition(clamp((1 + rotationPos) * 0.5, HORIZONTAL_ROTATION_MIN, HORIZONTAL_ROTATION_MAX));
//    }
    //WHY? ^

    public void setTargetState(GripperState state) {
        clawState = state;
        grabServos.setPosition(state.pos);
    }


    public boolean isSamplePresent(){
        boolean state = proximityFilter.estimate(proximitySensor.isPressed() ? 1 : 0) > CLAW_PROXIMITY_THRESHOLD;
        //logger.info("SAMPLE PRESENT SATE: " + state);
        return state;
    }

    public void disable() {
        angleServo.getController().pwmDisable();
        twistServo.getController().pwmDisable();
        grabServos.getController().pwmDisable();
    }

    @Override
    public void periodic(){
        FtcDashboard.getInstance().getTelemetry().addData("CLAW PROXIMITY STATE SENSOR: ", isSamplePresent());
    }
}