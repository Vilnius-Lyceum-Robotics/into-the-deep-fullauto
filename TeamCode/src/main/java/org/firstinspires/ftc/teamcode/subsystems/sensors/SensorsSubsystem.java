package org.firstinspires.ftc.teamcode.subsystems.sensors;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SensorsSubsystem extends VLRSubsystem<SensorsSubsystem> implements SensorsConfiguration{

    private static final Logger log = LoggerFactory.getLogger(SensorsSubsystem.class);
    AnalogInput backSensor;
    AnalogInput frontSensor;
    AnalogInput leftSensor;
    AnalogInput rightSensor;
    AnalogInput leftAngledSensor;
    AnalogInput rightAngledSensor;

    LowPassFilter rightSensorFilter = new LowPassFilter(0.5);
    LowPassFilter leftSensorFilter = new LowPassFilter(0.5);
    LowPassFilter backSensorFilter = new LowPassFilter(0.5);
    LowPassFilter rightAngledSensorFilter = new LowPassFilter(0.5);
    LowPassFilter leftAngledSensorFilter = new LowPassFilter(0.5);
    LowPassFilter frontSensorFilter = new LowPassFilter(0.5);

    private double backDistance;
    private double frontDistance;
    private double rightDistance;
    private double leftDistance;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        backSensor = hardwareMap.get(AnalogInput.class, BACK_SENSOR);
        frontSensor = hardwareMap.get(AnalogInput.class, FRONT_SENSOR);
        leftSensor = hardwareMap.get(AnalogInput.class, LEFT_SENSOR);
        rightSensor = hardwareMap.get(AnalogInput.class, RIGHT_SENSOR);
        leftAngledSensor = hardwareMap.get(AnalogInput.class, LEFT_ANGLED_SENSOR);
        rightAngledSensor = hardwareMap.get(AnalogInput.class, RIGHT_ANGLED_SENSOR);
    }

    @Override
    public void periodic(){
        updateFrontDistance();
        updateBackDistance();
        updateLeftDistance();
        updateRightDistance();

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("LEFT DISTANCE: ", leftDistance);
        telemetry.addData("RIGHT DISTANCE: ", rightDistance);
        telemetry.addData("BACK DISTANCE: ", backDistance);
        telemetry.addData("FRONT DISTANCE: ", frontDistance);
    }

    public void updateBackDistance(){
        backDistance = backSensorFilter.estimate(backSensor.getVoltage() / 3.3 * 100);
    }
    public void updateFrontDistance() {
        frontDistance = frontSensorFilter.estimate(frontSensor.getVoltage() / 3.3 * 100);
    }

    public void updateLeftDistance() {
        leftDistance = leftSensorFilter.estimate(leftSensor.getVoltage() / 3.3 * 100);
    }

    public void updateRightDistance() {
        rightDistance = rightSensorFilter.estimate(rightSensor.getVoltage() / 3.3 * 100);
    }
}
