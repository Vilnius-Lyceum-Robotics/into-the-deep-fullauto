package org.firstinspires.ftc.teamcode.auto.specimen;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DELTA;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DEPOSIT_SAMPLE_3_END;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DEPOSIT_SAMPLE_3_START;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DRIVE_BACK;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.MIDPOINT_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SAMPLE_1;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SAMPLE_2;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SAMPLE_3;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_PRELOAD_AND_SUB_PICKUP;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_SECOND_SPECIMEN;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_SPECIMEN_BACK;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.START_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.commands.HoldPoint;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.SetPattern;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.RequestLimelightFrame;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.WaitUntilNextLimelightFrame;

import java.util.logging.Level;

public class AutonomousPeriodActionSpecimen extends SequentialCommandGroup {
    private volatile boolean readyForSubPickup = false;
    private final double pathTValueConstraint = 0.92;

    public AutonomousPeriodActionSpecimen(Follower follower, LimelightYoloReader reader) {
        addCommands(
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                new SetPattern().red(),

                //DRIVE TO BAR AND EXTEND ARM
                new ParallelCommandGroup(
                        new SetArmPosition().scoreSpecimenFront(),
                        new WaitCommand(110).andThen(new FollowPath(follower, bezierPath(START_POSE, SCORE_PRELOAD_AND_SUB_PICKUP)
                                        .setConstantHeadingInterpolation(SCORE_PRELOAD_AND_SUB_PICKUP.getHeading()).build()))
                ),

                new WaitCommand(200),
                new RequestLimelightFrame(reader, follower),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                new SetArmPosition().extensionRelative(0.21),
                                new InstantCommand(()-> readyForSubPickup = true),
                                new SetPattern().green()
                        ),

                        new SequentialCommandGroup(
                                new WaitUntilNextLimelightFrame(reader),
                                new WaitUntilCommand(()-> readyForSubPickup),
                                new SubmersibleGrabV2(follower, reader, true)
                        )
                ),


                //SCORING SPIKE MARK SAMPLE INTO HUMAN PLAYER AREA
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetPattern().oceanPalette(),
                                new InstantCommand(()-> VLRSubsystem.getArm().setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL_SLOWER)),
                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(140),
                                        new WaitCommand(500).andThen(new SetClawAngle(0.5))
                                ),
                                new WaitUntilCommand(()-> follower.getPose().getY() < PICK_UP_SAMPLE_1.getY() + 3),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetPattern().green(),
                                new InstantCommand(()-> VLRSubsystem.getArm().setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL)),

                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(2).andThen(new SetPattern().red()),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() < 25),
                                                new SetArmPosition().extension(0.34)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() > 0.09),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new SetArmPosition().angleDegrees(0),
                                                new WaitCommand(30)
                                        )
                                )
                        ),



                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new ScheduleRuntimeCommand(
                                        ()-> new FollowPath(follower, bezierPath(follower.getPose(), new Pose(follower.getPose().getX() - 3, follower.getPose().getY(), follower.getPose().getHeading()))
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build())),
                                new ScheduleRuntimeCommand(
                                        ()-> new FollowPath(follower, bezierPath(follower.getPose(), PICK_UP_SAMPLE_1)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build()))
                        )
                ),


                //FIRST SPIKE MARK SAMPLE
                new SetPattern().green(),
                new ParallelCommandGroup(
                        new WaitCommand(500).andThen(new SetPattern().red()),
                        scoreSampleIntoHumanPlayerArea(0.34, 2),
                        new WaitCommand(350).andThen(new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build()))
                ),

                //SECOND SPIKE MARK SAMPLE
                new SetPattern().green(),
                new ParallelCommandGroup(
                        new WaitCommand(500).andThen(new SetPattern().red()),
                        scoreSampleIntoHumanPlayerArea(0.4185, 3),
                        new WaitCommand(400).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build()))
                ),

                //THIRD SPIKE MARK SAMPLE
                new WaitCommand(60),
                new SetPattern().green(),
                new ParallelCommandGroup(
                        new WaitCommand(500).andThen(new SetPattern().red()),
                        scoreSampleIntoHumanPlayerArea(0, 0.35, 4),
                        new SequentialCommandGroup(
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_3, DEPOSIT_SAMPLE_3_START)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), DEPOSIT_SAMPLE_3_START.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build(), false),

                                new WaitCommand(260),
                                new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3_START, DEPOSIT_SAMPLE_3_END)
                                        .setConstantHeadingInterpolation(DEPOSIT_SAMPLE_3_END.getHeading()).build())
                        )
                ),

                new SetPattern().green(),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(80),
                scoreSecondSample(follower),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(30),
                                new FollowPath(follower, bezierPath(SCORE_SECOND_SPECIMEN, DRIVE_BACK)
                                        .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).build(), false),
                                new FollowPath(follower, bezierPath(DRIVE_BACK, MIDPOINT_BEFORE_PICKUP)
                                        .setLinearHeadingInterpolation(DRIVE_BACK.getHeading(), MIDPOINT_BEFORE_PICKUP.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build(), false),
                                new FollowPath(follower, bezierPath(MIDPOINT_BEFORE_PICKUP, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(MIDPOINT_BEFORE_PICKUP.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SetArmPosition().retract().andThen(new SetArmPosition().intakeSpecimen(0.44))
                ),

                cycle(follower, 3),
                cycle(follower, 4),
                cycle(follower, 5),
                cycle(follower, 6)
        );
    }


    private Command cycle(Follower follower, int sample){
        Pose targetStart = new Pose(SCORE_SPECIMEN_BACK.getX(), SCORE_SPECIMEN_BACK.getY() + (sample - 3) * DELTA, SCORE_SPECIMEN_BACK.getHeading());
        Pose targetEnd = new Pose(targetStart.getX() + 15, targetStart.getY(), targetStart.getHeading());

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, targetStart)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), targetStart.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build()),
                                new LogCommand("SPECIMEN PERIOD ACTIONS", Level.SEVERE, "SKIBIDI FIRST FOLLOW PATH PASSED"),
                                new FollowPath(follower, bezierPath(targetStart, targetEnd)
                                        .setConstantHeadingInterpolation(targetStart.getHeading()).build())
                                        .interruptOn(()-> VLRSubsystem.getInstance(Chassis.class).getBackDistance() < 362),

                                new InstantCommand(follower::breakFollowing),
                                new LogCommand("BACK DISTANCE LOGGER", Level.INFO, ()-> "BACK SENSOR DISTANCE: " + VLRSubsystem.getInstance(Chassis.class).getBackDistance()),
                                new ScheduleRuntimeCommand(()-> new HoldPoint(follower, follower.getPose())).withTimeout(70)
                        ),
                        new SetArmPosition().scoreSpecimenBack()
                ),

                new WaitCommand(60),
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SetArmPosition().extensionRelative(0.215),
                                        new SetPattern().green(),
                                        new SetArmPosition().intakeSpecimen(0.485)
                                ),


                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new InstantCommand(follower::resumePathFollowing),

                                        new ScheduleRuntimeCommand(()-> new FollowPath(follower, bezierPath(follower.getPose(), MIDPOINT_BEFORE_PICKUP)
                                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), MIDPOINT_BEFORE_PICKUP.getHeading()).build(), false)),

                                        new FollowPath(follower, bezierPath(MIDPOINT_BEFORE_PICKUP, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                                .setLinearHeadingInterpolation(MIDPOINT_BEFORE_PICKUP.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build()
                                        )
                                )
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().extensionRelative(0.215).andThen(new SetPattern().green()),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(100),
                                new SetArmPosition().extensionAndAngleDegrees(0, 96)
                        ),
                        ()-> sample <= 5
                )
        );
    }



    private Command scoreSecondSample(Follower follower){
        return new ParallelCommandGroup(
                new WaitCommand(50).andThen(
                        new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3_END, DRIVE_BACK)
                                .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build(), false),
                        new FollowPath(follower, bezierPath(DRIVE_BACK, SCORE_SECOND_SPECIMEN)
                                .setConstantHeadingInterpolation(SCORE_SECOND_SPECIMEN.getHeading()).build())
                ),
                new ParallelCommandGroup(
                        new SetArmPosition().angleDegrees(100).andThen(new SetArmPosition().extensionAndAngleDegrees(0.53 , 50)),
                        new WaitCommand(200).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN))
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(()-> follower.atPose(SCORE_SECOND_SPECIMEN, 1.6, 1.6, Math.toRadians(4))),
                        new SetArmPosition().extensionRelative(0.23),
                        new SetPattern().green(),
                        new SetArmPosition().setArmState(ArmState.State.SPECIMEN_SCORE_FRONT)
                )
        );
    }

    private Command scoreSampleIntoHumanPlayerArea(double extension, int sample){
        return scoreSampleIntoHumanPlayerArea(extension, ClawConfiguration.HorizontalRotation.NORMAL.pos, sample);
    }


    private Command scoreSampleIntoHumanPlayerArea(double extension, double twist, int sample){
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                        new WaitCommand(150),
                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                        new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                        new SetArmPosition().extension(0),
                        new SetArmPosition().setArmState(ArmState.State.IN_ROBOT)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() < 0.26),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SetArmPosition().angleDegrees(153),
                                        new CustomConditionalCommand(
                                                new ParallelCommandGroup(
                                                        new SetArmPosition().angleDegrees(2),
                                                        new SequentialCommandGroup(
                                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() < 45),
                                                                new ParallelCommandGroup(
                                                                        new SetArmPosition().extension(extension),
                                                                        new SetClawTwist(twist),
                                                                        new SequentialCommandGroup(
                                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() > clamp(extension - 0.25, 0.08 ,1)),
                                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                                                new WaitCommand(200),
                                                                                new SetArmPosition().angleDegrees(0),
                                                                                new WaitCommand(30)
                                                                        )
                                                                )
                                                        )
                                                ),
                                                ()-> sample != 4
                                        )
                                ),
                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 135).andThen(new SetClawState(ClawConfiguration.GripperState.OPEN)),
                                new CustomConditionalCommand(new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 120).andThen(new SetClawAngle(0.52)), ()-> sample == 4)
                        )
                )
        );
    }
}