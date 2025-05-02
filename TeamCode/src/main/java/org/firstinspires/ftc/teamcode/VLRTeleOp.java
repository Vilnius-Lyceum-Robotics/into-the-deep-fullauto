package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.START_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_0;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrab;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.controls.trigger.TriggerCtl;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;

import java.util.logging.Logger;

import pedroPathing.tuners.constants.FConstants;
import pedroPathing.tuners.constants.LConstants;

@TeleOp(name = "VLRTeleOp", group = "!TELEOP")
@Photon
public class VLRTeleOp extends VLRLinearOpMode {
    Follower f;
    CommandScheduler cs;
    boolean followerActive = false;
    LimelightYoloReader reader = new LimelightYoloReader();

    MainArmConfiguration.SAMPLE_SCORE_HEIGHT armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET;
    boolean slideResetActive = false;
    boolean rotatorResetActive = false;

    RumbleControls rc;

    @Override
    public void run() {
        LimelightYoloReader reader = new LimelightYoloReader();

        cs = CommandScheduler.getInstance();

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class, Wiper.class);
        VLRSubsystem.initializeAll(hardwareMap);

        f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        if (!PoseSaver.isPoseSaved()) {
            // Set default sample start pos
            f.setStartingPose(START_POSE);
            telemetry.addLine("No saved pose found, using default");
            telemetry.update();
        } else f.setStartingPose(PoseSaver.getPedroPose());

        GamepadEx gpEx = new GamepadEx(gamepad1);
        DriverControls gp = new DriverControls(gpEx);
        rc = new RumbleControls(gamepad1);

        gp.add(new ButtonCtl(GamepadKeys.Button.A, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> toggleFollower()));

        gp.add(new ButtonCtl(GamepadKeys.Button.X, ButtonCtl.Trigger.STATE_JUST_CHANGED, (Boolean a) -> {
            if (!slideResetActive) startSlideOverride();
            else endSlideOverride();
        }));
        gp.add(new ButtonCtl(GamepadKeys.Button.Y, ButtonCtl.Trigger.STATE_JUST_CHANGED, (Boolean a) -> {
            if (!rotatorResetActive) startRotatorOverride();
            else endRotatorOverride();
        }));

        gp.add(new ButtonCtl(GamepadKeys.Button.DPAD_LEFT, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> toggleArmLowState()));

        gp.add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> subGrab()));
        gp.add(new ButtonCtl(GamepadKeys.Button.RIGHT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> deposit()));

        gp.add(new TriggerCtl(GamepadKeys.Trigger.RIGHT_TRIGGER, (Double a) -> {
            if (a > 0.3) retractArm();
        }));

        // todo wipe and hang

        waitForStart();


        while (opModeIsActive()) {
            gp.update();


            if (followerActive) f.update();
            else {
                // Not defining these controls through DriverControls cuz ts pmo
                VLRSubsystem.getInstance(Chassis.class).drive(gpEx.getLeftY(), -gpEx.getLeftX(), -0.3 * gpEx.getRightX());
                f.updatePose();
            }
        }
    }

    //
    // FOLLOWER
    //
    private void toggleFollower() {
        followerActive = !followerActive;

        if (followerActive) {
            rc.doubleBlip();
            f.holdPoint(f.getPose());
        } else {
            rc.singleBlip();
        }
    }

    //
    // UTILS
    //
    private void wipe(double x) {
        // x here should be 0-1
        VLRSubsystem.getInstance(Wiper.class).wipe(x);
    }

    private void startSlideOverride() {
        Logger.getLogger("SlideOverride").fine("Start override");
        slideResetActive = true;
        VLRSubsystem.getArm().enableSlidePowerOverride(-0.3);
    }

    private void endSlideOverride() {
        Logger.getLogger("SlideOverride").fine("End override");
        slideResetActive = false;
        VLRSubsystem.getArm().disableSlidePowerOverride();
    }

    private void startRotatorOverride() {
        Logger.getLogger("RotatorOverride").fine("Start override");
        rotatorResetActive = true;
        VLRSubsystem.getArm().enableRotatorPowerOverride(-0.1);
    }

    private void endRotatorOverride() {
        Logger.getLogger("RotatorOverride").fine("End override");
        rotatorResetActive = false;
        VLRSubsystem.getArm().disableRotatorPowerOverride();
    }

    //
    // ARM OPS
    //
    private void retractArm() {
        cs.schedule(new SetArmPosition().retract());
    }

    private void toggleArmLowState() {
        if (armState == MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET) {
            armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.LOW_BASKET;
        } else {
            armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET;
        }
    }

    // TODO - HANG

    private void subGrab() {
        followerActive = true;
        f.holdPoint(new Pose(f.getPose().getX(), f.getPose().getY(), SUB_GRAB.getHeading()));
        double headingError = Math.abs(f.getPose().getHeading() - SUB_GRAB.getHeading());

        cs.schedule(
                new SequentialCommandGroup(
                        new LogCommand("SubGrabTeleop", "Heading error: " + headingError),
                        new WaitCommand((long) (headingError * 30)),
                        new SubmersibleGrab(f, Alliance.BLUE, reader, rc),
                        new WaitCommand(230),
                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                        new WaitCommand(150),
                        new SetArmPosition().retract()
                )
        );
    }

    private void deposit() {
        followerActive = true;
        cs.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SequentialCommandGroup(
                                        new SetArmPosition().retract(),
                                        new SetArmPosition().scoreSample(armState)
                                ),
                                new SequentialCommandGroup(
                                        new FollowPath(f, bezierPath(f.getPose(), SUB_GRAB_0, BUCKET_HIGH_SCORE_POSE)
                                                .setLinearHeadingInterpolation(SUB_GRAB.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()
                                        ),
                                        new InstantCommand() {
                                            @Override
                                            public void run() {
                                                followerActive = false;
                                                rc.singleBlip();
                                            }
                                        }
                                )
                        )

                )
        );
    }

}
