package org.firstinspires.ftc.teamcode.subsystems.sensors;

import android.graphics.PointF;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class SensorsSubsystem extends VLRSubsystem<SensorsSubsystem> implements SensorsConfiguration{

    private static final Logger logger = LoggerFactory.getLogger(SensorsSubsystem.class);

    AnalogInput backSensor;
    AnalogInput leftSensor;
    AnalogInput rightSensor;
    AnalogInput leftAngledSensor;
    AnalogInput rightAngledSensor;

    LowPassFilter rightSensorFilter = new LowPassFilter(0.5);
    LowPassFilter leftSensorFilter = new LowPassFilter(0.5);
    LowPassFilter backSensorFilter = new LowPassFilter(0.3);
    LowPassFilter rightAngledSensorFilter = new LowPassFilter(0.97);
    LowPassFilter leftAngledSensorFilter = new LowPassFilter(0.97);

    private double backDistance;
    private double rightDistance;
    private double leftDistance;
    private double rightAngledDistance;
    private double leftAngledDistance;

    private Pose backPose;
    private Pose rightPose;
    private Pose leftPose;
    private Pose rightAngledPose;
    private Pose leftAngledPose;

    List<PointF> field;

    public static double sensorScalar = 1105;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        backSensor = hardwareMap.get(AnalogInput.class, BACK_SENSOR);
        leftSensor = hardwareMap.get(AnalogInput.class, LEFT_SENSOR);
        rightSensor = hardwareMap.get(AnalogInput.class, RIGHT_SENSOR);
        leftAngledSensor = hardwareMap.get(AnalogInput.class, LEFT_ANGLED_SENSOR);
        rightAngledSensor = hardwareMap.get(AnalogInput.class, RIGHT_ANGLED_SENSOR);

        field = SUBMERSIBLE_POLYGON;
    }

    @Override
    public void periodic(){
        updateBackDistance();
        updateLeftDistance();
        updateRightDistance();

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("LEFT DISTANCE: ", leftDistance);
        telemetry.addData("RIGHT DISTANCE: ", rightDistance);
        telemetry.addData("BACK DISTANCE: ", backDistance);
    }

    public void updateBackDistance(){
        backDistance = backSensorFilter.estimate(backSensor.getVoltage() / 3.3 * 31.5);
    }

    public void updateLeftDistance() {
        leftDistance = leftSensorFilter.estimate(leftSensor.getVoltage() / 3.3 * 31.5);
    }

    public void updateRightDistance() {
        rightDistance = rightSensorFilter.estimate(rightSensor.getVoltage() / 3.3 * 31.5);
    }

    public void updateLeftAngledDistance() {
        leftAngledDistance = leftAngledSensorFilter.estimate(leftAngledSensor.getVoltage() / 3.3 * sensorScalar);
    }

    public void updateRightAngledDistance() {
        rightAngledDistance = rightAngledSensorFilter.estimate(rightAngledSensor.getVoltage() / 3.3 * sensorScalar);
    }

    public void updateBackPose(Pose currentRobotPose) {
        if (backDistance >= 800) return;
        backPose = new Pose(
                currentRobotPose.getX() - backDistance * Math.cos(currentRobotPose.getHeading()),
                currentRobotPose.getY() - backDistance * Math.sin(currentRobotPose.getHeading())
            );
    }

    public Pose detectedBackObjectPos(Follower follower) {
        if (backDistance >= 3.94 ) return null;
        logger.info("DETECTED OBJECT IN THE FRONT");
        Pose currentRobotPose = follower.getPose();
        double object_X = currentRobotPose.getX() - backDistance * Math.cos(currentRobotPose.getHeading());
        double object_Y = currentRobotPose.getY() - backDistance * Math.sin(currentRobotPose.getHeading());

        if (isObjectField(new Pose(object_X, object_Y))) {
            logger.info("FIELD IN FRONT, IGNORING");
            return null;
        }

        logger.info("OTHER ROBOT AT " + object_X + ", " + object_Y);
        return new Pose(object_X, object_Y);
    }

    // TODO add the remaining sensors
    public List<Pose> detectObstacles(Follower follower) {
        List<Pose> foreignObjectPositions = new ArrayList<>();
        Pose currentRobotPose = follower.getPose();

        updateBackPose(currentRobotPose);

        if (!isObjectField(backPose)) foreignObjectPositions.add(backPose);

        return foreignObjectPositions;
    }

    public boolean isObjectField(Pose objectPose) {
        if (objectPose == null) return true;
        int crossings = 0;
        int size = field.size();

        for (int i = 0; i < size; i++) {

            PointF a = field.get(i);
            PointF b = field.get((i + 1) % size);

            if (((a.y > objectPose.getY()) != (b.y > objectPose.getY()))) {

                double slope = (b.x - a.x) * (objectPose.getY() - a.y) / (b.y - a.y) + a.x;
                if (objectPose.getX() < slope) {
                    crossings++;
                }
            }
        }

        return (crossings % 2 == 1);
    }
}
