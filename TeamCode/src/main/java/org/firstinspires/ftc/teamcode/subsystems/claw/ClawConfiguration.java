package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ClawConfiguration {
    String ANGLE_SERVO = "angle";
    String TWIST_SERVO = "twist";
    String GRAB_SERVO = "claw";
    String ANALOG_PROXIMITY = "clawAnalogProximity";

    double HORIZONTAL_ROTATION_MIN = 0;
    double HORIZONTAL_ROTATION_MAX = 1;

    double CLAW_ANALOG_PROXIMITY_THRESHOLD = 0.92;//1.065;

    enum VerticalRotation {
        DOWN(1),
        UP(0.105),
        DEPOSIT(0.4);

        public final double pos;
        VerticalRotation(double pos) {
            this.pos = pos;
        }
    }

    enum HorizontalRotation {
        NORMAL(0.5);

        public final double pos;

        HorizontalRotation(double pos) {
            this.pos = pos;
        }
    }

    enum GripperState {
        CLOSED(0.1),
        CLOSED_LOOSE(0.145),
        OPEN(0.92);

        public final double pos;

        GripperState(double pos) {
            this.pos = pos;
        }
    }
}