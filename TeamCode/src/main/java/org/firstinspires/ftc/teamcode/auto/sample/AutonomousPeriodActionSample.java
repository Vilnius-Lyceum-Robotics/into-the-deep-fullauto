package org.firstinspires.ftc.teamcode.auto.sample;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.*;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.logging.Level;

public class AutonomousPeriodActionSample extends SequentialCommandGroup {
    private boolean sampleScored = false;
    private Alliance alliance;
    private LimelightYoloReader reader;
    private ElapsedTime autoTimer = new ElapsedTime();
    private double elapsedTime = 0;
    private double jointPathTValue = 0.5;

    public AutonomousPeriodActionSample(Follower follower, Alliance alliance, LimelightYoloReader reader) {
        this.alliance = alliance;
        this.reader = reader;

        addCommands(
                new org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand() {
                    @Override
                    public void run() {
                        autoTimer = new ElapsedTime();
                    }
                },
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //SCORE PRELOAD AND PICK UP FIRST SAMPLE FROM SPIKE MARK
                scorePreload(follower),

                //GRAB FIRST SPIKE MARK, DRIVE TO BUCKET, DRIVE TO SECOND
                grabAndScoreSpikeMarkSample(follower, 2),

                //GRAB SECOND SPIKE MARK, DRIVE TO BUCKET, DRIVE TO THIRD
                grabAndScoreSpikeMarkSample(follower, 3),

                //GRAB THIRD SPIKE MARK, DRIVE TO BUCKET, DRIVE TO SUB GRAB POSE
                grabAndScoreSpikeMarkSample(follower, 4),

                subCycle(follower, 5),
                subCycle(follower, 6),
                subCycle(follower, 7),

                new ScheduleRuntimeCommand(() -> new LogCommand("SKIBIDI AUTO TIME SECONDS: ", Level.SEVERE, "SKIBIDI AUTO TIME SECONDS: " + elapsedTime))
        );
    }


    private Command scorePreload(Follower follower) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),
                        new WaitUntilCommand(() -> follower.atPose(BUCKET_HIGH_SCORE_POSE, 2, 2, Math.toRadians(3))),
                        new InstantCommand(() -> sampleScored = true),
                        new SetArmPosition().intakeSample(0.34)
//                        new ParallelCommandGroup(
//                                new SetArmPosition().retract(),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(200),
//                                        new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
//                                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() < 15),
//                                        new SetArmPosition().intakeSample(0.34)
//                                )
//                        )
                ),

                new SequentialCommandGroup(
                        new FollowPath(follower, bezierPath(START_POSE, BUCKET_HIGH_SCORE_POSE)
                                .setLinearHeadingInterpolation(START_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()
                        ),
                        new WaitUntilCommand(() -> sampleScored),
                        new InstantCommand(() -> sampleScored = false),
                        new WaitCommand(200),
                        new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, FIRST_MARK_GRAB)
                                .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), FIRST_MARK_GRAB.getHeading()).build()
                        )
                )
        );
    }


    private Command subCycle(Follower follower, int sample) {
        return new SequentialCommandGroup(
                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                new SubmersibleGrab(follower, alliance, reader),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),

                                new org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand() {
                                    @Override
                                    public void run() {
                                        if (sample == 7) {
                                            elapsedTime = autoTimer.seconds();
                                        }
                                    }
                                },

                                new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),
                                new WaitUntilCommand(() -> follower.atPose(BUCKET_HIGH_SCORE_POSE, 2, 2, Math.toRadians(2.5))),
                                new InstantCommand(() -> sampleScored = true),
                                new SetArmPosition().retract()
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(120),
                                new FollowPath(follower, bezierPath(SUB_GRAB, SUB_GRAB_CONTROL_2, SUB_GRAB_CONTROL_1, SUB_GRAB_0)
                                        .setTangentHeadingInterpolation().setReversed(true).build(), false).setCompletionThreshold(jointPathTValue),
                                new LogCommand("Auto bombo", "Passed 1st path"),
                                new FollowPath(follower, bezierPath(SUB_GRAB_0, BUCKET_HIGH_SCORE_POSE_SUB)
                                        .setConstantHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading()).build()),
                                new LogCommand("Auto bombo", "Passed 2nd path"),

                                new WaitUntilCommand(() -> sampleScored),
                                new InstantCommand(() -> sampleScored = false),
                                new WaitCommand(200),

                                new CustomConditionalCommand(
                                        new SequentialCommandGroup(
                                                new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE_SUB, SUB_GRAB_0)
                                                        .setConstantHeadingInterpolation(SUB_GRAB_0.getHeading()).build(), false).setCompletionThreshold(jointPathTValue),
                                                new FollowPath(follower, bezierPath(SUB_GRAB_0, SUB_GRAB_CONTROL_1, SUB_GRAB_CONTROL_2, SUB_GRAB)
                                                        .setTangentHeadingInterpolation().build())
                                        ),

                                        () -> sample <= 6
                                )
                        )
                )
        );
    }


    private Command grabAndScoreSpikeMarkSample(Follower follower, int sample) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetArmPosition().retract(),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> VLRSubsystem.getArm().currentExtension() < 0.15),
                                        new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                                        new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET)
                                )
                        ),
                        new WaitUntilCommand(() -> follower.atPose(BUCKET_HIGH_SCORE_POSE, 4, 4, Math.toRadians(5))),
                        new InstantCommand(() -> sampleScored = true),
                        new SetArmPosition().setArmState(ArmState.State.SAMPLE_SCORE),

                        new ConditionalCommand(
                                new SetArmPosition().intakeSampleAuto(0.31, sample == 4 ? 0.3 : 0.5),
                                new SetArmPosition().retract().andThen(new SetClawState(ClawConfiguration.GripperState.OPEN)),
                                () -> sample <= 3
                        )
                ),


                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new FollowPath(follower, bezierPath(GRAB_POSES[sample - 2], BUCKET_HIGH_SCORE_POSE)
                                .setLinearHeadingInterpolation(GRAB_POSES[sample - 2].getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()),

                        new WaitUntilCommand(() -> sampleScored),
                        new InstantCommand(() -> sampleScored = false),

                        new WaitCommand(100),
                        new ConditionalCommand(
                                new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, sample != 4 ? GRAB_POSES[sample - 1] : new Pose())
                                        .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), sample != 4 ? GRAB_POSES[sample - 1].getHeading() : new Pose().getHeading()).build()),

                                new SequentialCommandGroup(
                                        new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, SUB_GRAB_0)
                                                .setConstantHeadingInterpolation(SUB_GRAB_0.getHeading()).build(), false).setCompletionThreshold(jointPathTValue),
                                        new FollowPath(follower, bezierPath(SUB_GRAB_0, SUB_GRAB_CONTROL_1, SUB_GRAB_CONTROL_2, SUB_GRAB)
                                                .setTangentHeadingInterpolation().build())
                                ),

                                () -> sample <= 3
                        )

                )
        );
    }
}