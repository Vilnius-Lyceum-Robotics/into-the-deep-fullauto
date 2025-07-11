package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.EXCLUSION_ZONE_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.EXCLUSION_ZONE_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.EXCLUSION_ZONE_MIN_EXTENSION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.ROBOT_LENGTH_CM;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.SAMPLE_SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_EXTENSION_CM;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

import java.util.logging.Level;

public class MainArmSubsystem extends VLRSubsystem<MainArmSubsystem> {
    private ArmRotatorSubsystem rotator;
    private ArmSlideSubsystem slides;

    private Point targetPoint = new Point(0, 0);
    private Point prevTargetPoint = new Point(0, 0);

    private OPERATION_MODE operationMode = OPERATION_MODE.NORMAL;

    private SAMPLE_SCORE_HEIGHT sampleScoreHeight = SAMPLE_SCORE_HEIGHT.HIGH_BASKET;
    private boolean readyToProceedToLevel3 = false;

    protected void initialize(HardwareMap hardwareMap){
        readyToProceedToLevel3 = false;

        rotator = new ArmRotatorSubsystem(hardwareMap);
        slides = new ArmSlideSubsystem(hardwareMap);

        if (ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)){
            targetPoint = new Point(slides.getTargetExtension(), rotator.getTargetPosition());
            prevTargetPoint = targetPoint;
        }
    }

    public void enableAfterEndBrake(HardwareMap hardwareMap) {
        //afterEndBrake = new Thread(new AfterEndThread(hardwareMap));
        //afterEndBrake.start();
    }

    private class AfterEndThread implements Runnable {
        private HardwareMap hardwareMap;

        public AfterEndThread(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }
        @Override
        public void run() {
            while (true) {
                hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardwareMap.get(DcMotorEx.class, ArmRotatorConfiguration.MOTOR_NAME).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                System.out.println("After hang, overriding power");
            }
        }
    }

    public void setTargetPoint(Point point) {
        targetPoint = point;
        //logger.log(Level.WARNING, "NEW TARGET UPDATED IN ARM SUBSYSTEM WITH ANGLE " + point.angleDegrees() + " AND MAGNITUDE " + point.magnitude());
    }

    public void setTargetPoint(double magnitude, double angleDegrees) {
        setTargetPoint(new Point(magnitude, angleDegrees));
    }

    public void setRotatorPowerLimit(double powerLimit){
        rotator.setPowerLimit(powerLimit);
    }

    public void setSlidePowerLimit(double powerLimit){
        slides.setPowerLimit(powerLimit);
    }

    public void setThirdSlideMotorEnable(boolean state){
        slides.setThirdMotorEnable(state);
    }

    public Point calculateTargetPointFromRealWordCoordinates(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference){
        double minY = ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER.getY() + RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.getY();
        if (y_cm < minY) {
            y_cm = minY;
            logger.log(Level.SEVERE, "TARGET Y COORDINATE IS OUT OF BOUNDS, CLAMPING TO MIN VALUE");
        }

        Vector2d endEffectorFromPivotReferencePoint = new Vector2d(
                reference.xScalar * (x_cm + ROBOT_LENGTH_CM / 2 + Math.abs(ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER.getX())),
                y_cm - Math.abs(ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER.getY())
        );

        logger.log(Level.WARNING, "END EFFECTOR FROM PIVOT POINT ANGLE: " + Math.toDegrees(endEffectorFromPivotReferencePoint.angle()) + " ;MAGNITUDE: " + endEffectorFromPivotReferencePoint.magnitude());

        double alpha = Math.acos(Math.abs(RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.getY() / endEffectorFromPivotReferencePoint.magnitude()));
        double beta = Math.atan(RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.getX() / Math.abs(RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.getY()));
        double sigma = endEffectorFromPivotReferencePoint.angle();
        double gamma = beta - alpha + sigma;

        logger.log(Level.WARNING, "ALPHA: " + Math.toDegrees(alpha));
        logger.log(Level.WARNING, "BETA: " + Math.toDegrees(beta));
        logger.log(Level.WARNING, "SIGMA: " + Math.toDegrees(sigma));
        logger.log(Level.WARNING, "GAMMA: " + Math.toDegrees(gamma));

        double magnitude = RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.magnitude();
        Vector2d retractedArmEndFactor = new Vector2d(magnitude * Math.cos(gamma), magnitude * Math.sin(gamma));

        logger.log(Level.WARNING, "RETRACTED END EFFECTOR VECTOR: " + Math.toDegrees(retractedArmEndFactor.angle()) + "; " + retractedArmEndFactor.magnitude());

        Vector2d extensionVector = endEffectorFromPivotReferencePoint.minus(retractedArmEndFactor);
        extensionVector = extensionVector.scale(1 / MAX_EXTENSION_CM);

        double extension = extensionVector.magnitude();
        double angleDegrees = Math.toDegrees(extensionVector.angle());

        if (extension < 0 || extension > 1 || angleDegrees < MIN_ANGLE || angleDegrees > MAX_ANGLE){
            logger.log(Level.SEVERE, "TARGET POINT:  " + x_cm + "; " + y_cm + " IS UNREACHABLE, EITHER USER IS AN IDIOT, OR THERE IS A BUG IN ARM KINEMATICS");
            logger.log(Level.SEVERE, "DEFAULTING TO THE CLOSEST ACHIEVABLE POINT");

            angleDegrees = clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE);
            extension = clamp(extension, 0, 1);
        }

        logger.log(Level.WARNING, "SETTING ARM TO " + extension + " " + angleDegrees + " TO ACHIEVE TARGET X AND Y REAL WORLD POSITION");
        return new Point(extension, angleDegrees);
    }

    public Point getTargetPoint() {return targetPoint;}

    public void proceedToLevel3() {readyToProceedToLevel3 = true;}

    public boolean isReadyToProceedToLevel3() {return readyToProceedToLevel3;}

    public double getTargetAngleDegrees() {return clamp(targetPoint.angleDegrees(), 0, 180);}

    public double getPrevTargetAngleDegrees() {return clamp(prevTargetPoint.angleDegrees(), 0, 180);}

    public double getTargetAngleRads() {return clamp(targetPoint.angleRads(), 0, Math.PI);}

    public double getTargetExtension() {return clamp(targetPoint.magnitude(), 0, 1);}

    public double getTargetX() {return clamp(targetPoint.getX(), -1, 1);}

    public  double getTargetY() {return clamp(targetPoint.getY(), 0, 1);}

    public void setOperationMode(OPERATION_MODE operationMode) {
        this.operationMode = operationMode;
    }

    public boolean isCurrentOperationMode(OPERATION_MODE operationMode){
        return this.operationMode == operationMode;
    }

    public boolean isCurrentOperationMode(OPERATION_MODE... operationModes){
        for (OPERATION_MODE operation_mode : operationModes){
            if (this.operationMode == operation_mode){
                return true;
            }
        }
        return false;
    }

    public void setSampleScoreHeight(SAMPLE_SCORE_HEIGHT sampleScoreHeight){
        this.sampleScoreHeight = sampleScoreHeight;
    }

    public SAMPLE_SCORE_HEIGHT getSampleScoreHeight() {return sampleScoreHeight;}


    @Override
    public void periodic(){
        if (!prevTargetPoint.equals(targetPoint)){
            rotator.setTargetPosition(targetPoint.angleDegrees(), slides.getExtension(), operationMode);
            slides.setTargetPosition(targetPoint.magnitude());
            prevTargetPoint = targetPoint;
        }

        rotator.periodic(targetPoint.magnitude(), operationMode);
        slides.periodic(targetPoint.angleDegrees(), operationMode);
    }

    public boolean isBetween(double num, double num1, double num2){
        return num > Math.min(num1, num2) && num < Math.max(num1, num2);
    }

    public boolean isTargetPointValid(double targetAngle, double magnitude){
        return !(isBetween(targetAngle, EXCLUSION_ZONE_MIN_ANGLE, EXCLUSION_ZONE_MAX_ANGLE) && magnitude > EXCLUSION_ZONE_MIN_EXTENSION);
    }

    public boolean isCameraInTheWayToNewTarget(Point target){
        double prevTarget = targetPoint.angleDegrees();
        boolean minBool = isBetween(EXCLUSION_ZONE_MIN_ANGLE, prevTarget, target.angleDegrees());
        boolean maxBool = isBetween(EXCLUSION_ZONE_MAX_ANGLE, prevTarget, target.angleDegrees());
        boolean prevExt = targetPoint.magnitude() > EXCLUSION_ZONE_MIN_EXTENSION;
        boolean currentExt = target.magnitude() > EXCLUSION_ZONE_MIN_EXTENSION;

        boolean state = (minBool || maxBool) && (prevExt || currentExt);

        if (state){
            logger.log(Level.SEVERE, "CAMERA IS IN THE WAY, INDIVIDUAL BOOLEANS ARE " + minBool + maxBool + prevExt + currentExt);
            logger.log(Level.SEVERE, "PREV TARGET: " + targetPoint.angleDegrees() + "; "  + targetPoint.magnitude());
            logger.log(Level.SEVERE, "CURRENT TARGET: " + target.angleDegrees() + "; " + target.magnitude());
        }
        return state;
    }

    public boolean reachedTargetPosition(){
        return rotator.reachedTargetPosition() && slides.reachedTargetPosition();
    }

    public boolean motionProfilePathsAtParametricEnd(){
        return rotator.getT() == 1 && slides.getT() == 1;
    }

    public boolean isRotatorMoving() {return rotator.getT() != 1;}

    public boolean areSlidesMoving() {return slides.getT() != 1;}

    public void enableSlidePowerOverride(double power) {slides.enablePowerOverride(power);}

    public void disableSlidePowerOverride() {slides.disablePowerOverride();}

    public void setRotatorTargetIllegal(double angleDeg) {rotator.setTargetPosition(angleDeg, 0, OPERATION_MODE.NORMAL);}

    public void enableRotatorPowerOverride(double power) {rotator.enablePowerOverride(power);}

    public void disableRotatorPowerOverride() {rotator.disablePowerOverride();}

    public void resetRotatorEncoder() {rotator.resetEncoder();}

    public double currentExtension() {return slides.getExtension();}

    public double currentAngleDegrees() {return rotator.getAngleDegrees();}

    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }
}