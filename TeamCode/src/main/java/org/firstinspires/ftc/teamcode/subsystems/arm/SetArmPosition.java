package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem.getArm;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.SAMPLE_SCORE_HEIGHT;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.commands.SetHangPosition;

import java.util.function.BooleanSupplier;
import java.util.logging.Level;

public class SetArmPosition extends SequentialCommandGroup{
    private final MainArmSubsystem arm;
    private static BooleanSupplier interruptCondition = ()-> true;

    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        //addRequirements(getArm());
    }

    public SetArmPosition() {arm = getArm();}


    private Command safeSetTargetPoint(Point targetPoint, Command ifCameraIsInTheWay){
        return new ConditionalCommand(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SET ARM POS COMMAND", "SETTING TARGET ARM ANGLE TO " + targetPoint.angleDegrees() + " WITH MAGNITUDE " + targetPoint.magnitude()),
                                new InstantCommand(()-> arm.setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL)),
                                new InstantCommand(()-> arm.setTargetPoint(targetPoint)),
                                new WaitCommand(5),
                                new WaitUntilCommand(arm::motionProfilePathsAtParametricEnd),

                                new InstantCommand(()-> arm.setOperationMode(MainArmConfiguration.OPERATION_MODE.HOLD_POINT)),

                                new ConditionalCommand(
                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET REACHED SUCCESSFULLY"),
                                        new LogCommand("SET ARM POS COMMAND", Level.WARNING, "TARGET NOT REACHED"),
//                                        new SequentialCommandGroup(
//                                                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM TIMEOUT TRIGGERED, WAITING INDEFINITELY FOR MANUAL OVERRIDE"),
//                                                new WaitUntilCommand(arm::reachedTargetPosition).interruptOn(interruptCondition),
//                                                new ConditionalCommand(
//                                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "WAIT INTERRUPTED, HOPE THE DRIVER KNOWS WHAT HES DOING"),
//                                                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "ARM MAGICALLY REACHED TARGET POSITION WITH NO OVERRIDE"),
//                                                        interruptCondition
//                                                )
//                                        ),
                                        arm::reachedTargetPosition
                                )
                        ),

                        ifCameraIsInTheWay,
                        ()-> !arm.isCameraInTheWayToNewTarget(targetPoint)
                ),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "TARGET POINT IS INVALID, SKIPPING"),
                ()-> arm.isTargetPointValid(targetPoint.angleDegrees())
        );
    }


    private Command safeSetTargetPoint(Point point, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return safeSetTargetPoint(point, moveToTargetWhileAvoidingCamera(point, gamePieceType));
    }

    public Command extensionAndAngleDegrees(double magnitude, double angleDegrees, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return safeSetTargetPoint(new Point(magnitude, angleDegrees), gamePieceType);
    }

    public Command extensionAndAngleDegrees(double magnitude, double angleDegrees){
        return safeSetTargetPoint(new Point(magnitude, angleDegrees), MainArmConfiguration.GAME_PIECE_TYPE.SPECIMEN);
    }

    public Command XY(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(
                ()-> new ConditionalCommand(
                        new LogCommand("SET ARM POSITION.XY", Level.SEVERE, "ARM ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH")
                                .andThen(new WaitUntilCommand(arm::motionProfilePathsAtParametricEnd).interruptOn(interruptCondition)
                        ),
                        safeSetTargetPoint(arm.calculateTargetPointFromRealWordCoordinates(x_cm, y_cm, reference), gamePieceType),
                        ()-> !arm.motionProfilePathsAtParametricEnd()
                )
        );
    }


    private Command moveToTargetWhileAvoidingCamera(Point target, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new SequentialCommandGroup(
                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "MAKING A CAMERA AVOIDANCE MANEUVER"),

                new ScheduleRuntimeCommand(
                        ()-> safeSetTargetPoint(new Point(0, arm.getTargetAngleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG ON RETRACTION"))
                ),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, ()-> ("FIRST SECTION PASSED, NOW GOING TO: " + target.angleDegrees() + "; 0")),

                new CustomConditionalCommand(
                        new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                        ()-> gamePieceType == MainArmConfiguration.GAME_PIECE_TYPE.SPECIMEN
                ),

                new ScheduleRuntimeCommand(
                        ()-> safeSetTargetPoint(new Point(0, target.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE ROTATING TO TARGET"))
                ),


                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, ()-> ("SECOND SECTION PASSED, NOW GOING TO: " + target.angleDegrees() + target.magnitude())),

                new ScheduleRuntimeCommand(
                        ()-> safeSetTargetPoint(new Point(target.magnitude(), target.angleDegrees()),
                        new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "SOMETHING WENT WRONG WHILE EXTENDING ARM TO TARGET")
                )),

                new LogCommand("SET ARM POS COMMAND", Level.SEVERE, "END OF CAMERA AVOIDANCE MANEUVER")
        );
    }

    public Command extension(double extension){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new LogCommand("SET ARM POSITION.EXTENSION", Level.SEVERE, "SLIDES ALREADY MOVING"),
                        ()-> (arm.areSlidesMoving() && arm.getTargetExtension() != extension)
                ),

                new ScheduleRuntimeCommand(()-> new SetArmPosition().extensionAndAngleDegrees(extension, arm.getTargetAngleDegrees()))

        );
    }

    public Command extensionRelative(double delta){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new LogCommand("SET ARM POSITION.EXTENSION", Level.SEVERE, "SLIDES ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH"),
                        ()-> (arm.areSlidesMoving() && arm.getTargetExtension() != arm.getTargetExtension() + delta)
                ),

                new ScheduleRuntimeCommand(()-> new SetArmPosition().extensionAndAngleDegrees(arm.getTargetExtension() + delta, arm.getTargetAngleDegrees()))
        );
    }


    public Command angleDegrees(double angleDegrees){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        new LogCommand("SET ARM POSITION.ANGLE_DEGREES", Level.SEVERE, "ROTATOR ALREADY MOVING, WAITING PREVIOUS COMMAND TO FINISH"),
                        ()-> (arm.isRotatorMoving() && arm.getTargetAngleDegrees() != angleDegrees)
                ),

                new ScheduleRuntimeCommand(()-> new SetArmPosition().extensionAndAngleDegrees(arm.getTargetExtension(), angleDegrees))
        );
    }

    public Command X(double x, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(() -> {
            double currentY = arm.getTargetY();
            return new SetArmPosition().XY(x, currentY, reference, gamePieceType);
        });
    }

    public Command Y(double y, OFFSET_REFERENCE_PLANE reference, MainArmConfiguration.GAME_PIECE_TYPE gamePieceType){
        return new ScheduleRuntimeCommand(() -> {
            double currentX = arm.getTargetX();
            return new SetArmPosition().XY(currentX, y, reference, gamePieceType);
        });
    }

    public Command MANUALLY_INTERRUPT_CURRENT_COMMAND(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> interruptCondition = ()-> true),
                new WaitCommand(20),
                new InstantCommand(()-> interruptCondition = ()-> false)
        );
    }

    public Command SET_INTERRUPT_CONDITION(BooleanSupplier state){
        return new InstantCommand(()-> interruptCondition = state);
    }

    public Command setArmState(ArmState.State state){
        return new InstantCommand(()-> ArmState.set(state));
    }


    private Command intake(double extension, double angle, double twist){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.SAMPLE_SCORE)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN ROBOT STATE"),

                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(twist),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetArmPosition().extensionAndAngleDegrees(extension, 2).alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() > clamp(extension - 0.25, 0.08 ,1)),
                                                new SetClawAngle(angle)
                                        )
                                ),
                                new SetArmPosition().angleDegrees(0),
                                new WaitCommand(100)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("INTAKE SAMPLE COMMAND", "INTAKING SAMPLE FROM IN SAMPLE SCORE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(70),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(100),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(0, 52, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                        new WaitCommand(350).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

                                new WaitUntilCommand(()-> arm.currentExtension() < 0.1),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(2).andThen(new SetClawTwist(twist)),
                                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() < 40),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                                                new ParallelCommandGroup(
                                                        new SetArmPosition().extension(extension),
                                                        new SequentialCommandGroup(
                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                                                new WaitUntilCommand(()-> arm.currentExtension() > clamp(extension - 0.25, 0.08 ,1)),
                                                                new SetClawAngle(angle),
                                                                new WaitCommand(200),
                                                                new SetArmPosition().angleDegrees(0),
                                                                new WaitCommand(50)
                                                        )
                                                )
                                        )
                                )
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE)
                )
        );
    }

    public Command intakeSample(double extension) {
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, ClawConfiguration.HorizontalRotation.NORMAL.pos)
                .andThen(setArmState(ArmState.State.SAMPLE_INTAKE));
    }

    public Command intakeSampleAuto(double extension, double twist) {
        return intake(extension, ClawConfiguration.VerticalRotation.DOWN.pos, twist)
                .andThen(setArmState(ArmState.State.SAMPLE_INTAKE));
    }

    public Command intakeSpecimen(double extension) {
        return intake(extension, 0.88, ClawConfiguration.HorizontalRotation.NORMAL.pos)
                .andThen(setArmState(ArmState.State.SPECIMEN_INTAKE));
    }

    public Command retract(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                       new SequentialCommandGroup(
                               new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN SCORE STATE"),

                               new SetClawState(ClawConfiguration.GripperState.OPEN),
                               new WaitCommand(100),

                               new ParallelCommandGroup(
                                       new SequentialCommandGroup(
                                               new WaitCommand(80),
                                               new SetClawAngle(0.5),
                                               new WaitCommand(150),
                                               new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                               new SetClawAngle(ClawConfiguration.VerticalRotation.UP)
                                       ),
                                       new SequentialCommandGroup(
                                               new SetArmPosition().extension(0),
                                               new SetArmPosition().angleDegrees(0)
                                       )
                               ),
                               setArmState(ArmState.State.IN_ROBOT)
                       ),
                       ()-> ArmState.isCurrentState(ArmState.State.SPECIMEN_SCORE_FRONT)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SAMPLE SCORE STATE"),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(70),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(60),

                                new ParallelCommandGroup(
                                        new SetArmPosition().extensionAndAngleDegrees(0, 52, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE),
                                        new WaitCommand(250).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

                                new WaitUntilCommand(()-> arm.currentExtension() < 0.2),
                                new WaitUntilCommand(()-> arm.currentExtension() < 0.08).withTimeout(1000),
                                new SetArmPosition().angleDegrees(0),
                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM ANY HANG STATE"),

                                new CustomConditionalCommand(
                                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),
                                        ()-> ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
                                ),

                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(80),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(50),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(65),
                                        new WaitCommand(100).andThen(new SetArmPosition().extension(0)),
                                        new WaitCommand(250).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                                ),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new SetArmPosition().angleDegrees(0),
                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE, ArmState.State.HANG_THIRD_STAGE)
                ),


                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("RETRACT ARM", Level.SEVERE, "RETRACTING ARM FROM SPECIMEN OR SAMPLE INTAKE STATE"),

                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(170),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetArmPosition().extension(0),

                                setArmState(ArmState.State.IN_ROBOT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE, ArmState.State.SPECIMEN_INTAKE)
                )
        );
    }


    public Command scoreSample(SAMPLE_SCORE_HEIGHT sampleScoreHeight){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SAMPLE", Level.SEVERE, "SCORING SAMPLE FROM IN ROBOT STATE"),

                                new ParallelCommandGroup(
                                        new WaitCommand(300).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)),
                                        new SetArmPosition().angleDegrees(101),
                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 17).andThen(new SetArmPosition().extension(sampleScoreHeight.extension)),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentExtension() > sampleScoreHeight.extension - 0.27),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT)
                                        )
                                ),
                                setArmState(ArmState.State.SAMPLE_SCORE)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                )
        );
    }


    public Command scoreSpecimenFront(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SPECIMEN FRONT", Level.SEVERE, "SCORING SPECIMEN FRONT FROM IN ROBOT STATE"),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new SetArmPosition().extensionAndAngleDegrees(0, 62),
                                                new SetArmPosition().extensionAndAngleDegrees(0.53, 49.5)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 40),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                                new WaitCommand(250),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)
                                        )
                                ),
                                setArmState(ArmState.State.SPECIMEN_SCORE_FRONT)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                )
        );
    }


    public Command scoreSpecimenBack(){
        return new SequentialCommandGroup(
                new CustomConditionalCommand(
                        retract(),
                        ()-> !ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new LogCommand("SCORE SPECIMEN BACK", Level.SEVERE, "SCORING SPECIMEN BACK FROM IN ROBOT STATE"),
                                new SetArmPosition().extensionAndAngleDegrees(0.3, 102).alongWith(
                                        new WaitUntilCommand(()-> arm.currentAngleDegrees() > 80).andThen(new SetClawAngle(0.265))
                                ),
                                setArmState(ArmState.State.SPECIMEN_SCORE_BACK)
                        ),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                )
        );
    }


    public Command level_2_hang(BooleanSupplier gamepadCondition){
        return new CustomConditionalCommand(
                new SequentialCommandGroup(
                        new LogCommand("SECOND STAGE HANG", Level.SEVERE, "STARTING LEVEL 2 HANG COMMAND"),

                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                        new SetArmPosition().angleDegrees(102.5).alongWith(
                                new WaitUntilCommand(()-> arm.currentAngleDegrees() >= 70).andThen(new SetArmPosition().extension(0.314))
                        ),

                        new WaitUntilCommand(gamepadCondition),
                        new SetArmPosition().extension(0.1),
                        new InstantCommand(()-> arm.updateCoefficients(OPERATION_MODE.HANG)),

                        new SetArmPosition().angleDegrees(42),
                        new SetArmPosition().extension(0.03),

                        setArmState(ArmState.State.HANG_SECOND_STAGE)
                ),
                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
        );
    }


    public Command level_3_hang(BooleanSupplier gamepadCondition){
        return new CustomConditionalCommand(
                new SequentialCommandGroup(
                        new LogCommand("THIRD STAGE HANG", Level.SEVERE, "STARTING LEVEL 3 HANG COMMAND"),

                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                        new InstantCommand(()-> VLRSubsystem.getHang().setTargetAngleUP()),
                        new SetArmPosition().angleDegrees(101).alongWith(
                                new WaitUntilCommand(()-> arm.currentAngleDegrees() > 80).andThen(new SetArmPosition().extension(0.314))
                        ),

                        new WaitUntilCommand(gamepadCondition),
                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),

                        new ParallelCommandGroup(
                                new SetArmPosition().extension(0.15),
                                new WaitCommand(200).andThen(new SetArmPosition().angleDegrees(85)),
                                new WaitCommand(500).andThen(new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.2)))
                        ).interruptOn(()-> VLRSubsystem.getHang().analogFeedbackThresholdReached()),

                        new InstantCommand(()-> VLRSubsystem.getHang().setPower(0)),
                        new SetArmPosition().extension(0.3),

                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.NORMAL)),
                        new SetArmPosition().extension(0.888).alongWith(new SetArmPosition().angleDegrees(80)),

                        new SetArmPosition().angleDegrees(102).alongWith(
                                new WaitCommand(400).andThen(new SetArmPosition().extension(0.83))
                        ),

                        new WaitUntilCommand(gamepadCondition),
                        new InstantCommand(()-> arm.setOperationMode(OPERATION_MODE.HANG)),

                        new SetArmPosition().extension(0.06).alongWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(()-> arm.currentExtension() < 0.3),
                                        new SetHangPosition(HangConfiguration.TargetPosition.DOWN),
                                        new SetArmPosition().angleDegrees(35)
                                )
                        )
                ),
                ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
        );
    }
}