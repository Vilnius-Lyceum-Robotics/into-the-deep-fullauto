package org.firstinspires.ftc.teamcode.auto.specimen;//package org.firstinspires.ftc.teamcode.commands.specimen;

import static org.firstinspires.ftc.teamcode.auto.specimen.Points_specimen.*;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.logging.Level;

public class AutonomousPeriodActionSpecimen extends SequentialCommandGroup {
    private boolean firstSpikeMarkSamplePickedUp = false;
    private boolean secondSpikeMarkSamplePickedUp = false;
    private boolean thirdSpikeMarkSamplePickedUP = false;

    public AutonomousPeriodActionSpecimen(Follower follower, Alliance alliance, LimelightYoloReader reader) {
        addCommands(
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //DRIVE TO BAR AND EXTEND ARM
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().scoreSpecimenFront(),
                                new WaitUntilCommand(()-> follower.getPose().getX() > 35.25).andThen(new SetArmPosition().extensionRelative(0.17))
                        ),

                        new FollowPath(follower, bezierPath(START_POSE, SCORE_PRELOAD_AND_SUB_PICKUP)
                                .setConstantHeadingInterpolation(SCORE_PRELOAD_AND_SUB_PICKUP.getHeading()).build())
                ),

                //SCORING SPIKE MARK SAMPLE INTO HUMAN PLAYER AREA
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().retract().alongWith(
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() < 15),
                                                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                                                new SetArmPosition().intakeSample(0.35)
                                        )
                                ),

                                new WaitUntilCommand(()-> follower.getPose().getY() < 15),
                                new InstantCommand(()-> firstSpikeMarkSamplePickedUp = true),

                                new SetArmPosition().setArmState(ArmState.State.SAMPLE_INTAKE),
                                scoreSampleIntoHumanPlayerArea().andThen(new SetArmPosition().intakeSample(0.35)),

                                new WaitUntilCommand(()-> follower.getPose().getY() < 6),
                                new InstantCommand(()-> this.secondSpikeMarkSamplePickedUp = true),
                                scoreSampleIntoHumanPlayerArea().andThen(new SetArmPosition().intakeSample(0.43)),


                                new WaitUntilCommand(()-> (follower.getPose().getY() < 5 && follower.headingError < Math.toRadians(5))),
                                new InstantCommand(()-> this.thirdSpikeMarkSamplePickedUP = true),

                                new SetArmPosition().retract(),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(155),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 140),
                                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                                new SetClawAngle(0.52)
                                        )
                                )
                        ),


                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new FollowPath(follower, bezierPath(SCORE_PRELOAD_AND_SUB_PICKUP, PICK_UP_SAMPLE_1)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).build()),


                                new WaitUntilCommand(()-> firstSpikeMarkSamplePickedUp),
                                new WaitCommand(150),
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).build()),

                                new WaitUntilCommand(()-> secondSpikeMarkSamplePickedUp),
                                new WaitCommand(150),
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).build()),

                                new WaitUntilCommand(()-> thirdSpikeMarkSamplePickedUP),
                                new WaitCommand(150),
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_3, DEPOSIT_SAMPLE_3_START)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), DEPOSIT_SAMPLE_3_START.getHeading()).build()),

                                new WaitCommand(200),
                                new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3_START, DEPOSIT_SAMPLE_3_END)
                                        .setConstantHeadingInterpolation(DEPOSIT_SAMPLE_3_END.getHeading()).build())
                        )
                ),

                new WaitCommand(100),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(120),

                new ParallelCommandGroup(
                        new WaitCommand(50).andThen(
                                new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3_END, DRIVE_BACK)
                                        .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).build()).setCompletionThreshold(0.9),
                                new FollowPath(follower, bezierPath(DRIVE_BACK, SCORE_SECOND_SPECIMEN)
                                        .setConstantHeadingInterpolation(SCORE_SECOND_SPECIMEN.getHeading()).build())
                        ),
                        new ParallelCommandGroup(
                                new SetArmPosition().angleDegrees(100).andThen(new SetArmPosition().extensionAndAngleDegrees(0.53 , 50)),
                                new WaitCommand(200).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN))
                        ),
                        new WaitUntilCommand(()-> follower.getPose().getX() > 33.5).andThen(new SetArmPosition().extensionRelative(0.23)).andThen(new SetArmPosition().setArmState(ArmState.State.SPECIMEN_SCORE_FRONT))
                ),



                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(30),
                                new FollowPath(follower, bezierPath(SCORE_SECOND_SPECIMEN, DRIVE_BACK)
                                        .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).build()).setCompletionThreshold(0.9),
                                new FollowPath(follower, bezierPath(DRIVE_BACK, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(DRIVE_BACK.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new WaitCommand(400),
                                new SetArmPosition().intakeSpecimen(0.44)
                        )
                ),



                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, SCORE_SPECIMEN_BACK)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), SCORE_SPECIMEN_BACK.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().scoreSpecimenBack()
                        )
                ),

                new SetArmPosition().extensionRelative(0.21),

                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(SCORE_SPECIMEN_BACK, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(SCORE_SPECIMEN_BACK.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().intakeSpecimen(0.44)
                        )
                ),


                //SCORE FOURTH
                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, SCORE_SPECIMEN_BACK_4)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), SCORE_SPECIMEN_BACK_4.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().scoreSpecimenBack()
                        )
                ),

                new SetArmPosition().extensionRelative(0.21),

                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(SCORE_SPECIMEN_BACK_4, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(SCORE_SPECIMEN_BACK_4.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().intakeSpecimen(0.44)
                        )
                ),


                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, SCORE_SPECIMEN_BACK_5)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), SCORE_SPECIMEN_BACK_5.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().scoreSpecimenBack()
                        )
                ),

                new SetArmPosition().extensionRelative(0.21),
                new SetArmPosition().retract()
        );
    }



    private Command scoreSampleIntoHumanPlayerArea(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetArmPosition().retract().andThen(new LogCommand("SKIBIDI LOG", Level.SEVERE, "RETRACT PASSED")),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() < 0.15),
                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(150),
                                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 135).andThen(new SetClawState(ClawConfiguration.GripperState.OPEN))
                                )
                        )
                ),
                new SetArmPosition().angleDegrees(0)
        );
    }
}