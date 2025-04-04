package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSlideConfiguration {
    public static String MOTOR_NAME_0 = "MotorArm1";
    public static String MOTOR_NAME_1 = "MotorArm2";
    public static String MOTOR_NAME_2 = "MotorArm3";
    public static String ENCODER_NAME = "MotorLeftBack";

    public static String LIMIT_SW_NAME = "SlideLimit";

//    public static double ACCELERATION_JERK = 700000;
//    public static double DECELERATION_JERK = 480000;
//    public static double MAX_VELOCITY = 3100;
//    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.0255;
//    public static double FEEDBACK_INTEGRAL_GAIN = 0.000001;
//    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0004;
//    public static double FEED_FORWARD_GAIN = 0.1;
//    public static double VELOCITY_GAIN = 0.0002;
//    public static double ACCELERATION_GAIN = 0.000016;


    public static double ACCELERATION = 700;
    public static double DECELERATION = 500;
    public static double MAX_VELOCITY = 200;
    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.35;
    public static double FEEDBACK_INTEGRAL_GAIN = 0.06;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.018;
    public static double FEED_FORWARD_GAIN = 0.08;
    public static double VELOCITY_GAIN = 0.12;
    public static double ACCELERATION_GAIN = 0.08;


    public static double ACCELERATION_HANG = 70;
    public static double DECELERATION_HANG = 50;
    public static double MAX_VELOCITY_HANG = 20;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.6;
    public static double FEEDBACK_DERIVATIVE_GAIN_HANG = 0.013;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0.00001;
    public static double FEED_FORWARD_GAIN_HANG = -0.25;
    public static double VELOCITY_GAIN_HANG = 0;
    public static double ACCELERATION_GAIN_HANG = 0;


    public static double ACCELERATION_HANG_FAST = 0;
    public static double DECELERATION_HANG_FAST = 0;
    public static double MAX_VELOCITY_HANG_FAST = 0;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG_FAST = 0;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG_FAST = 0;
    public static double VELOCITY_GAIN_HANG_FAST = 0;


    public static double CREEP = 2;

    public static double ERROR_MARGIN = 0.3;
    public static double ERROR_TIMEOUT_MILLIS = 1000; // ms before position is automatically classified as reached

    public static double MIN_POSITION = 0;
    public static double MIN_MANUAL_ADJUST_POSITION = 3;
    public static double HORIZONTAL_EXTENSION_LIMIT = 15;
    public static double MAX_POSITION = 23; //ROTATIONS, NOT TICKS
    public static double TICKS_PER_IN = (6523 / 8192d) * 0.93;//17.67 * 2.54 * 1.1; // 53.36 //TODO: RECALC
    public static double MAX_EXTENSION_IN = HORIZONTAL_EXTENSION_LIMIT / TICKS_PER_IN;

    public static double INTAKE_SPECIMEN = 0.05;


    public enum TargetPosition {
        RETRACTED(0),
        INTAKE_SAMPLE(0.45), // 30.8cm
        SCORE_BUCKET_HIGH(1),
        PREPARE_SPECIMEN_HIGH(0.45),
        SCORE_SPECIMEN_HIGH(0.165),
        INTAKE_SPECIMEN(0.45);

        public final double extension;

        TargetPosition(double extension) {
            this.extension = extension;
        }
    }


    public enum OperationMode {
        NORMAL,
        HANG_SLOW,
        HANG_FAST
    }
}
