package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@Config
public class HangSubsystem extends VLRSubsystem<HangSubsystem> implements HangConfiguration {
    private CRServo left, right;
    private AnalogInput analogLeft, analogRight;

    public static double p = 0.005;
    private double powerOverride = 0;
    private boolean overridePower = true;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        left = hardwareMap.get(CRServo.class, LEFT_AXON);
        right = hardwareMap.get(CRServo.class, RIGHT_AXON);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        analogLeft = hardwareMap.get(AnalogInput.class, LEFT_ANALOG);
        analogRight = hardwareMap.get(AnalogInput.class, RIGHT_ANALOG);

        setPower(0);
    }


    @Override
    public void periodic(){
        double powerLeft = p * (getAngle(analogLeft.getVoltage()) - leftAnalogThreshold - 20);
        double powerRight = p * (rightAnalogThreshold - 20 - getAngle(analogRight.getVoltage()));

        if (overridePower) {
            left.setPower(powerOverride);
            right.setPower(powerOverride);
        }
        else {
            left.setPower(powerLeft);
            right.setPower(powerRight);
        }



        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        telemetry.addData("HANG_LEFT_ANALOG_ANGLE: ", getAngle(analogLeft.getVoltage()));
        telemetry.addData("HANG_RIGHT_ANALOG_ANGLE: ", getAngle(analogRight.getVoltage()));
        telemetry.addData("HANG_ANALOG_THRESHOLD_STATE: ", analogFeedbackThresholdReached() ? 1 : 0);
    }


    public void setPower(double power){
        overridePower = true;
        powerOverride = power;
    }


    public void setTargetAngleUP(){
        overridePower = false;
    }


    public double getAngle(double voltage){ return voltage / 3.3 * 360;}

    public double getLeftAngle(){ return getAngle(analogLeft.getVoltage());}
    public double getRightAngle(){ return getAngle(analogRight.getVoltage());}


    public boolean analogFeedbackThresholdReached(){
        return (
                getAngle(analogLeft.getVoltage()) < leftAnalogThreshold &&
                getAngle(analogRight.getVoltage()) > rightAnalogThreshold
        );
    }
}