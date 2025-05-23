package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.localization.Pose;

@Config
public interface ChassisConfiguration {
    String MOTOR_LEFT_FRONT = "MotorLeftFront";
    String MOTOR_RIGHT_FRONT = "MotorRightFront";
    String MOTOR_LEFT_BACK = "MotorLeftBack";
    String MOTOR_RIGHT_BACK = "MotorRightBack";

    String LEFT_ANGLED_SENSOR = "CHanalog2";
    String RIGHT_ANGLED_SENSOR = "CHanalog1";
    String BACK_SENSOR = "backAnalog";

    double DISTANCE_BETWEEN_ANGLED_SENSORS_MM = 303.9089; //mm
    Pose BUCKET_CORNER = new Pose(8.45, 144, 0); //inches
    Vector2d OFFSET_FROM_SENSOR_MIDPOINT_TO_PEDRO_CENTER = new Vector2d(76, 0); //13.3883, 0); //mm
}
