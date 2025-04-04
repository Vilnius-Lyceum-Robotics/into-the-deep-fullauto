package org.firstinspires.ftc.teamcode.helpers.utils;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// Generic motion profile generator and power calculator
// If given jerk will create a jerk limited motion profile or acceleration limited motion profile
// All profiles can be asymmetric
// Just don't look at the math, it's fucked up

public class MotionProfile {
    private final Telemetry telemetry;

    private double velocityGain, accelerationGain;
    private double maxVelocity, acceleration, deceleration;
    private double jerkAcceleration, jerkDeceleration;

    private double targetPosition;
    private double prevTargetPosition;
    private double initialPosition;
    private double initialTime;

    private double creep;

    private final String telemetryName;
    private boolean isTelemetryEnabled = false;

    private final PIDController pid;

    private Type profileType;


    public MotionProfile(Telemetry telemetry, String telemetryName, Type profileType, double acceleration, double deceleration, double maxVelocity, double creep, double p, double i, double d, double v, double a) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.profileType = profileType;

        switch (profileType){
            case ACCELERATION_LIMITED:
                this.acceleration = acceleration;
                this.deceleration = deceleration;
                break;

            case JERK_LIMITED:
                this.jerkAcceleration = acceleration;
                this.jerkDeceleration = deceleration;
                break;
        }

        this.maxVelocity = maxVelocity;
        this.creep = creep;
        this.velocityGain = v;
        this.accelerationGain = a;
        this.telemetryName = telemetryName;
        this.pid = new PIDController(p, i, d);
    }


    public enum Type{
        ACCELERATION_LIMITED,
        JERK_LIMITED
    }


    public void updateCoefficients(double acceleration, double deceleration, double maxVelocity, double p, double i, double d, double v, double a) {
        this.maxVelocity = maxVelocity;
        this.pid.setPID(p, i, d);
        this.velocityGain = v;
        this.accelerationGain = a;

        switch (profileType){
            case ACCELERATION_LIMITED:
                this.acceleration = acceleration;
                this.deceleration = deceleration;
                break;

            case JERK_LIMITED:
                this.jerkAcceleration = acceleration;
                this.jerkDeceleration = deceleration;
                break;
        }
    }


    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }


    public void enableTelemetry(boolean enableTelemetry) {
        this.isTelemetryEnabled = enableTelemetry;
    }


    public double getTargetPosition() {
        return targetPosition;
    }


    public double getPower(double currentPosition) {
        if (targetPosition != prevTargetPosition) {
            prevTargetPosition = targetPosition;
            initialPosition = currentPosition;
            initialTime = System.nanoTime();
        }

        double positionError = targetPosition - initialPosition;
        MotionState motionState = new MotionState(0, 0, 0);

        double elapsedTime = (System.nanoTime() - initialTime) / Math.pow(10, 9);

        if (Math.abs(positionError) > creep){
            switch (profileType){
                case ACCELERATION_LIMITED:
                    motionState = computeAccelerationMotionState(Math.abs(positionError), elapsedTime);
                    break;

                case JERK_LIMITED:
                    motionState = computeJerkMotionState(Math.abs(positionError), elapsedTime);
                    break;
            }
        }
        else motionState = new MotionState(Math.abs(positionError), 0, 0);

        double positionSetPoint = initialPosition + Math.signum(positionError) * motionState.position;

        double positionPower = pid.calculate(currentPosition, positionSetPoint);
        double velocityPower = motionState.velocity * velocityGain * Math.signum(positionError);
        double accelerationPower = motionState.acceleration * accelerationGain * Math.signum(positionError);

        if (isTelemetryEnabled) {
            logTelemetry(
                    currentPosition,
                    positionSetPoint,
                    motionState,
                    positionPower,
                    velocityPower,
                    accelerationPower
            );
        }
        return positionPower + velocityPower + accelerationPower;
    }


    private double calculateDistance(double jerk, double time) {
        return jerk * Math.pow(time, 2) * time;
    }


    public MotionState computeJerkMotionState(double distance, double elapsed_time) {
        if (distance == 0) return new MotionState(0, 0, 0);

        // Calculate the time it takes to accelerate to max velocity
        //a = jt
        //v = at = jt^2 / 2
        //v_max = 2 * jt^2 / 2
        //t = V(v_max / j)

        //WHILE ACCELERATING
        double acceleration_dt = 2 * Math.sqrt(maxVelocity / jerkAcceleration);
        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        //s1 = jt^3 /6
        //s2 = s1 + vt + at2 / 2 - jt3 / 6
        //s1 + s1 = vt + at^2 /2

        //s1 = jt^3 / 6
        //s2 = v0t + a0t^2 - jt^3

        double acceleration_distance = calculateDistance(jerkAcceleration, acceleration_dt / 2);

        //WHILE DECELERATING
        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        //s1 = jt^3 /6
        //s2 = s1 + vt + at2 / 2 - jt3 / 6
        //s1 + s1 = vt + at^2 /2
        double deceleration_dt = 2 * Math.sqrt(maxVelocity / jerkDeceleration);
        double deceleration_distance = calculateDistance(jerkDeceleration, deceleration_dt / 2);

        if (acceleration_distance + deceleration_distance > distance) {
            //recalculate acceleration dt
            //no cruise, just accelerate and decelerate
            //s = jt^3 / 6 + jt^3 / 6
            //s = 1/6 * (j1t^3 + j2t^3)
            //j1t1^2 = j2t2^2
            //t2 = V(j1t1^2 / j2)
            //t2 = v(j1 / j2) * t1
            //6s = j1t1^3 + j1t1^2 * t2
            //6s = j1t1^3 + j1t1^3 * V(j1 / j2)
            //j1t1^3(1 + V(j1 / j2) = 6s
            //t1^3 = 6s / (j1 * (1 + V(j1 / j2))

            acceleration_dt = Math.cbrt(6 * distance / (jerkAcceleration * (1 + Math.sqrt(jerkAcceleration / jerkDeceleration))));
        }

        acceleration_distance = calculateDistance(jerkAcceleration, acceleration_dt / 2);
        // recalculate max velocity based on the time we have to accelerate and decelerate
        //v = at^2 / 2
        double maxAchievableVelocity = jerkAcceleration * Math.pow(acceleration_dt / 2, 2);
        //v = v_max - jt^2
        //jt^2 = v_max
        //t = V(v_max / j)
        //recalculate deceleration
        deceleration_dt = 2 * Math.sqrt(maxAchievableVelocity / jerkDeceleration);
        deceleration_distance = calculateDistance(jerkDeceleration, deceleration_dt / 2);

        // calculate the time that we're at max velocity
        double cruise_distance = distance - acceleration_distance - deceleration_distance;
        double cruise_dt = cruise_distance / maxAchievableVelocity;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) return new MotionState(distance, 0, 0);

        double targetPosition, targetVelocity, targetAcceleration;

        // if we're accelerating
        if (elapsed_time < acceleration_dt / 2) {
            //jerk positive, acceleration positive
            targetAcceleration = jerkAcceleration * elapsed_time;
            targetVelocity = jerkAcceleration * Math.pow(elapsed_time, 2) / 2;
            targetPosition = jerkAcceleration * Math.pow(elapsed_time, 3) / 6;

            return new MotionState(targetPosition, targetVelocity, targetAcceleration, jerkAcceleration);


        } else if (elapsed_time < acceleration_dt) {
            //jerk negative, acceleration positive
            double current_dt = elapsed_time - acceleration_dt / 2;
            double baseAcceleration = jerkAcceleration * acceleration_dt / 2;
            double baseVelocity = jerkAcceleration * Math.pow(acceleration_dt / 2, 2) / 2;
            double basePosition = jerkAcceleration * Math.pow(acceleration_dt / 2, 3) / 6;

            targetAcceleration = jerkAcceleration * (acceleration_dt / 2) - jerkAcceleration * (current_dt);
            targetVelocity = baseVelocity + baseAcceleration * current_dt - jerkAcceleration * Math.pow(current_dt, 2) / 2;
            targetPosition = basePosition + baseVelocity * current_dt + baseAcceleration * Math.pow(current_dt, 2) / 2 - jerkAcceleration * Math.pow(current_dt, 3) / 6;

            return new MotionState(targetPosition, targetVelocity, targetAcceleration, -jerkAcceleration);
        }


        // if we're cruising
        else if (elapsed_time < acceleration_dt + cruise_dt) {
            targetPosition = acceleration_distance + maxAchievableVelocity * (elapsed_time - acceleration_dt);

            return new MotionState(targetPosition, maxAchievableVelocity, 0, 0);
        }


        // if we're decelerating
        else if (elapsed_time < acceleration_dt + cruise_dt + deceleration_dt / 2) {
            //jerk negative, acceleration negative
            double current_dt = elapsed_time - acceleration_dt - cruise_dt;
            targetAcceleration = -jerkDeceleration * (current_dt);
            targetVelocity = maxAchievableVelocity - jerkDeceleration * Math.pow(current_dt, 2) / 2;
            targetPosition = acceleration_distance + cruise_distance + maxAchievableVelocity * current_dt - jerkDeceleration * Math.pow(current_dt, 3) / 6;

            return new MotionState(targetPosition, targetVelocity, targetAcceleration, -jerkDeceleration);


        } else {
            //jerk positive, acceleration negative
            double current_dt = elapsed_time - acceleration_dt - cruise_dt - deceleration_dt / 2;
            double baseVelocity = maxAchievableVelocity - jerkDeceleration * Math.pow(deceleration_dt / 2, 2) / 2;
            double baseAcceleration = -jerkDeceleration * (deceleration_dt / 2);
            double basePosition = acceleration_distance + cruise_distance + maxAchievableVelocity * deceleration_dt / 2 - jerkDeceleration * Math.pow(deceleration_dt / 2, 3) / 6;

            targetPosition = basePosition + baseVelocity * current_dt + baseAcceleration * Math.pow(current_dt, 2) / 2 + jerkDeceleration * Math.pow(current_dt, 3) / 6;
            targetVelocity = baseVelocity + baseAcceleration * current_dt + jerkDeceleration * Math.pow(current_dt, 2) / 2;
            targetAcceleration = -jerkDeceleration * (deceleration_dt / 2) + jerkDeceleration * (current_dt);

            return new MotionState(targetPosition, targetVelocity, targetAcceleration, jerkDeceleration);
        }
    }


    public MotionState computeAccelerationMotionState(double distance, double elapsed_time) {
        if (distance == 0) return new MotionState(0, 0, 0);

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = maxVelocity / acceleration;
        double deceleration_dt = maxVelocity / deceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
        double deceleration_distance = 0.5 * deceleration * Math.pow(deceleration_dt, 2);

        if (acceleration_distance + deceleration_distance > distance) {
            acceleration_dt = Math.sqrt(2 * distance / (acceleration * (1 + acceleration / deceleration)));
        }

        acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        double maxAchievableVelocity = acceleration * acceleration_dt;

        deceleration_dt = maxAchievableVelocity / deceleration;
        deceleration_distance = 0.5 * deceleration * Math.pow(deceleration_dt, 2);

        // calculate the time that we're at max velocity
        double cruise_distance = distance - acceleration_distance - deceleration_distance;
        double cruise_dt = cruise_distance / maxAchievableVelocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return new MotionState(distance, 0, 0);
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            return new MotionState(0.5 * acceleration * Math.pow(elapsed_time, 2), acceleration * elapsed_time, acceleration);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            return new MotionState(acceleration_distance + maxAchievableVelocity * cruise_current_dt, maxAchievableVelocity, 0);
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = maxAchievableVelocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            return new MotionState(acceleration_distance + cruise_distance + maxAchievableVelocity * deceleration_time - 0.5 * deceleration * Math.pow(deceleration_time, 2),
                    maxAchievableVelocity - deceleration * deceleration_time, deceleration);
        }
    }


    public void logTelemetry(
            double currentPosition,
            double positionSetPoint,
            MotionState motionState,
            double positionPower,
            double velocityPower,
            double accelerationPower
    ) {
        logTelemetry(
                currentPosition,
                positionSetPoint,
                motionState.velocity,
                motionState.acceleration,
                positionPower,
                velocityPower,
                accelerationPower
        );
    }

    public void logTelemetry(
            double currentPosition,
            double positionSetPoint,
            double velocitySetPoint,
            double accelerationSetPoint,
            double positionPower,
            double velocityPower,
            double accelerationPower
    ) {
        telemetry.addData(telemetryName + "_motionProfileCurrentPosition: ", currentPosition);
        telemetry.addData(telemetryName + "_motionProfileTargetPosition: ", positionSetPoint);
        telemetry.addData(telemetryName + "_motionProfileTargetVelocity: ", velocitySetPoint);
        telemetry.addData(telemetryName + "_motionProfileTargetAcceleration: ", accelerationSetPoint);
        telemetry.addData(telemetryName + "_motionProfileTime: ", (System.nanoTime() - initialTime) / Math.pow(10, 9));
        telemetry.addData(telemetryName + "_motor power: ", clamp(positionPower + velocityPower + accelerationPower, -1, 1));
        telemetry.update();
    }
}