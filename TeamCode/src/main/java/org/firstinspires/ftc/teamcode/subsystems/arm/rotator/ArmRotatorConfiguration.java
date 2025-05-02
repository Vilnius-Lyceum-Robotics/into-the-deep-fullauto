package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotatorConfiguration {

    public static String MOTOR_NAME = "MotorRotator";
    public static String ENCODER_NAME = "MotorRightBack";
    public static String BEAM_BREAK_NAME = "BeamBreak";

    //JERK PROFILE CONSTANTS:
//    public static double ACCELERATION_JERK = 10000;
//    public static double DECELERATION_JERK = 9000;
//    public static double MAX_VELOCITY = 100;
//
//    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.04;
//    public static double FEEDBACK_INTEGRAL_GAIN = 0.000;
//    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0005;
//    public static double VELOCITY_GAIN = 0.001;
//    public static double ACCELERATION_GAIN = 0.00002;
//    public static double FEEDFORWARD_GAIN = 0.05;

    //VERY SLOW, LOGIC TUNING CONSTANTS
    public static double ACCELERATION_JERK = 36000;
    public static double DECELERATION_JERK = 28000;
    public static double MAX_VELOCITY = 900;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.031;
    public static double FEEDBACK_INTEGRAL_GAIN = 0.00001;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.00048;
    public static double VELOCITY_GAIN = 0.0021;
    public static double ACCELERATION_GAIN = 0.00018;
    public static double FEEDFORWARD_GAIN = 0.08;

    public static double EXTENDED_ACCELERATION_JERK = 9000;
    public static double EXTENDED_DECELERATION_JERK = 3000;
    public static double EXTENDED_MAX_VELOCITY = 75;
    public static double EXTENDED_FEEDBACK_PROPORTIONAL_GAIN = 0.1;
    public static double EXTENDED_FEEDBACK_INTEGRAL_GAIN = 0.0001;
    public static double EXTENDED_FEEDBACK_DERIVATIVE_GAIN = 0.0045;
    public static double EXTENDED_VELOCITY_GAIN = 0.0012;
    public static double EXTENDED_ACCELERATION_GAIN = 0.0012;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.38;

    public static double ACCELERATION_HANG = 3000;
    public static double DECELERATION_HANG = 2500;
    public static double MAX_VELOCITY_HANG = 40;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.07;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0;//.0001;
    public static double FEEDFORWARD_GAIN_HANG = 0;//.1;

    public static double FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT = 0.14;
    public static double FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT = 0.001;
    public static double FEEDBACK_INTEGRAL_GAIN_HOLD_POINT = 0.0002;

    public static double ERROR_MARGIN = 10;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 180;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;
}
