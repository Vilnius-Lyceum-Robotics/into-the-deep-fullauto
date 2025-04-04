package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface HangConfiguration {
    String LEFT_AXON = "leftHang";
    String RIGHT_AXON = "rightHang";

    String LEFT_ANALOG = "leftAnalog";
    String RIGHT_ANALOG = "rightAnalog";

    double leftAnalogThreshold = 59;
    double rightAnalogThreshold = 295;

    enum TargetPosition {
        DOWN(0, 0),
        UP(0, 0);

        public final double angleLeft, angleRight;
        TargetPosition(double angleLeft, double angleRight) {
            this.angleLeft = angleLeft;
            this.angleRight = angleRight;
        }
    }
}