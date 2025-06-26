package org.firstinspires.ftc.teamcode.subsystems.sensors;

import android.graphics.PointF;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;

@Config
public interface SensorsConfiguration {
    String BACK_SENSOR = "backAnalog";
    String LEFT_ANGLED_SENSOR = "CHanalog2";
    String RIGHT_ANGLED_SENSOR = "CHanalog1";
    String LEFT_SENSOR = "";
    String RIGHT_SENSOR = "";
    List<PointF> SUBMERSIBLE_POLYGON = new ArrayList<PointF>() {{
            new PointF(46, 98);
            new PointF(50, 98);
            new PointF(50, 88);
            new PointF(94, 88);
            new PointF(94, 98);
            new PointF(98, 98);
            new PointF(98, 46);
            new PointF(94, 46);
            new PointF(94, 56);
            new PointF(50, 56);
            new PointF(50, 46);
            new PointF(46, 46);
    }};

    double DISTANCE_BETWEEN_ANGLED_SENSORS = 303.9089; //mm
    double DISTANCE_BETWEEN_SIDE_SENSORS = 0;
    double DISTANCE_BETWEEN_BACK_SENSOR_AND_CENTER = 0;
}
