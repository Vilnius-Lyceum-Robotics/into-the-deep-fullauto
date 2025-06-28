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

    LowPassFilter rightSensorFilter = new LowPassFilter(0.97);
    LowPassFilter leftSensorFilter = new LowPassFilter(0.97);
    LowPassFilter backSensorFilter = new LowPassFilter(0.97);
    LowPassFilter rightAngledSensorFilter = new LowPassFilter(0.97);
    LowPassFilter leftAngledSensorFilter = new LowPassFilter(0.97);

    private double backDistance;
    private double rightDistance;
    private double leftDistance;
    private double rightAngledDistance;
    private double leftAngledDistance;

    private List<Pose> sensorPoses;

    List<PointF> field;

    public static double sensorScalar = 1105 / 25.4;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        backSensor = hardwareMap.get(AnalogInput.class, BACK_SENSOR);
        leftSensor = hardwareMap.get(AnalogInput.class, LEFT_SENSOR);
        rightSensor = hardwareMap.get(AnalogInput.class, RIGHT_SENSOR);
        leftAngledSensor = hardwareMap.get(AnalogInput.class, LEFT_ANGLED_SENSOR);
        rightAngledSensor = hardwareMap.get(AnalogInput.class, RIGHT_ANGLED_SENSOR);

        field = SUBMERSIBLE_POLYGON;

        sensorPoses = new ArrayList<>() {{
            new Pose(0, 0); // back sensor
            new Pose(0, 0); // right sensor
            new Pose(0, 0); // left sensor
        }};
    }

    @Override
    public void periodic(){
        updateBackDistance();
        updateLeftDistance();
        updateRightDistance();
    }
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("LEFT DISTANCE: ", leftDistance);
        telemetry.addData("RIGHT DISTANCE: ", rightDistance);
        telemetry.addData("BACK DISTANCE: ", backDistance);
    }

    public double getLeftDistance() {
        return this.leftDistance;
    }
    public double getRightDistance() {
        return this.rightDistance;
    }
    public double getBackDistance() {
        return this.backDistance;
    }

    public void updateBackDistance(){
        backDistance = backSensorFilter.estimate(backSensor.getVoltage() / 3.3 * 800);
    }

    public void updateLeftDistance() {
        leftDistance = leftSensorFilter.estimate(leftSensor.getVoltage() / 3.3 * 800);
    }

    public void updateRightDistance() {
        rightDistance = rightSensorFilter.estimate(rightSensor.getVoltage() / 3.3 * 800);
    }

    public void updateLeftAngledDistance() {
        leftAngledDistance = leftAngledSensorFilter.estimate(leftAngledSensor.getVoltage() / 3.3 * sensorScalar);
    }

    public void updateRightAngledDistance() {
        rightAngledDistance = rightAngledSensorFilter.estimate(rightAngledSensor.getVoltage() / 3.3 * sensorScalar);
    }

    public void updateBackPose(Pose currentRobotPose) {
        if (backDistance == 31.5) {
            sensorPoses.set(0, null);
            return;
        }
        sensorPoses.set(0, new Pose(
                currentRobotPose.getX() - (DISTANCE_BETWEEN_BACK_SENSOR_AND_CENTER + backDistance) * Math.cos(currentRobotPose.getHeading()),
                currentRobotPose.getY() - (DISTANCE_BETWEEN_BACK_SENSOR_AND_CENTER + backDistance) * Math.sin(currentRobotPose.getHeading())
            ));

        logger.info("BACK POSE UPDATED" + sensorPoses.get(0));
    }

    public void updateRightPose(Pose currentRobotPose) {
        if (rightDistance == 31.5) {
            sensorPoses.set(1, null);
            return;
        }
        sensorPoses.set(1, new Pose(
                currentRobotPose.getX() + (0.5 * DISTANCE_BETWEEN_SIDE_SENSORS + rightDistance) * Math.sin(currentRobotPose.getHeading()),
                currentRobotPose.getY() - (0.5 * DISTANCE_BETWEEN_SIDE_SENSORS + rightDistance) * Math.cos(currentRobotPose.getHeading())
        ));
        logger.info("RIGHT POSE UPDATED" + sensorPoses.get(1));
    }

    public void updateLeftPose(Pose currentRobotPose) {
        if (leftDistance == 31.5) {
            sensorPoses.set(2, null);
            return;
        }
        sensorPoses.set(2, new Pose(
                currentRobotPose.getX() - (0.5 * DISTANCE_BETWEEN_SIDE_SENSORS + leftDistance) * Math.sin(currentRobotPose.getHeading()),
                currentRobotPose.getY() + (0.5 * DISTANCE_BETWEEN_SIDE_SENSORS + leftDistance) * Math.cos(currentRobotPose.getHeading())
        ));
        logger.info("LEFT POSE UPDATED" + sensorPoses.get(2));
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

    // TODO add the angled sensors
    public List<Pose> getFilteredObstacles(Follower follower) {
        logger.info("GETTING FILTERED OBSTACLES...");
        List<Pose> foreignObjectPositions = new ArrayList<>();

        for (Pose pose : sensorPoses) {
            if (!isObjectField(pose)) foreignObjectPositions.add(pose);
        }

        return foreignObjectPositions;
    }

    public void updateAllPoses(Follower follower) {
        Pose currentRobotPose = follower.getPose();

        updateBackPose(currentRobotPose);
        updateRightPose(currentRobotPose);
        updateLeftPose(currentRobotPose);
    }
    public List<Pose> getAllObstacles(Follower follower) {
        logger.info("GETTING ALL OBSTACLES...");
        List<Pose> obstaclePoses = new ArrayList<>();
        updateAllPoses(follower);

        for (Pose pose : sensorPoses) {
            if (pose != null) obstaclePoses.add(pose);
        }

        return obstaclePoses;
    }

    public void printAllObstacles(Follower follower) {
        logger.info("PRINTING OBSTACLE POSITIONS...");
        List<Pose> obstaclePoses = this.getAllObstacles(follower);

        for (Pose pose : obstaclePoses) {
            System.out.println(pose.getX() + " " + pose.getY());
        }
    }

    public boolean isObjectField(Pose objectPose) {
        logger.info("FIELD DETECTION STARTED");
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

        logger.info("IS OBJECT FIELD: " + (crossings % 2 == 1));

        return (crossings % 2 == 1);
    }
}
