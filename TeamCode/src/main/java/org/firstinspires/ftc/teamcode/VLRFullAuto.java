//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.BUCKET_HIGH_SCORE_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.BUCKET_HIGH_SCORE_TRUE_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.SUB_GRAB_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.SUB_PRE_BEZIER_POSE;
//import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.SUB_PRE_PREGRAB_POSE;
//import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.pedropathing.commands.FollowPath;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commands.sample.AutonomousPeriodActions;
//import org.firstinspires.ftc.teamcode.commands.sample.HighBasketScore;
//import org.firstinspires.ftc.teamcode.commands.sample.SubmersibleGrab;
//import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
//import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
//import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
//import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
//import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.ArmLowState;
//import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ResetRotatorMotor;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
//import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
//import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
//import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
//import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
//import org.firstinspires.ftc.teamcode.subsystems.hang.commands.SecondStageHangCommand;
//import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
//import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;
//
//import java.util.ArrayList;
//import java.util.List;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//@TeleOp(name = "VLRFullAuto", group = "!TELEOP")
//@Photon
//public class VLRFullAuto extends VLRLinearOpMode {
//    Alliance alliance = Alliance.BLUE;
//
//    CommandScheduler cs;
//
//    Follower f;
//    Pose startPose = new Pose(10, 111.5, 0);
//
//    /**
//     * @noinspection unchecked
//     */
//    @Override
//    public void run() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//
//        cs = CommandScheduler.getInstance();
//        GameStateController gameStateController = new GameStateController(this::getTimeSinceInit, false); // FORCING TELEOP FOR TESTING
//        StrategyController strategyController = new StrategyController(gameStateController);
//        LimelightYoloReader reader = new LimelightYoloReader();
//
//        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, Chassis.class, Wiper.class);
//        VLRSubsystem.initializeAll(hardwareMap);
//
//        f = new Follower(hardwareMap);
//        f.setStartingPose(startPose);
//
//        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);
//        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
//                new AutoConfigurator.Choice("Red"),
//                new AutoConfigurator.Choice("Blue"));
//
//        AutoConfigurator.Choice autoChoice = configurator.multipleChoice("Run auto:",
//                new AutoConfigurator.Choice("Run"),
//                new AutoConfigurator.Choice("Do not run"));
//
//
//        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;
//
//        List<LimelightYoloReader.Limelight.Sample.Color> allowedColors = new ArrayList<>();
//        allowedColors.add(LimelightYoloReader.Limelight.Sample.Color.YELLOW);
//        allowedColors.add(alliance == Alliance.RED ? LimelightYoloReader.Limelight.Sample.Color.RED : LimelightYoloReader.Limelight.Sample.Color.BLUE);
//
//        reader.setAllowedColors(allowedColors);
//
//        boolean runAuto = autoChoice.text.equals("Run");
//
//        configurator.review("Alliance: " + alliance, "Auto: " + runAuto);
//
//        waitForStart();
//        //VLRSubsystem.getInstance(Limelight.class).setAlliance(Alliance.BLUE);
//        //VLRSubsystem.getInstance(Limelight.class).enable();
//        gameStateController.postInit();
//
//        GamepadEx gp = new GamepadEx(gamepad1);
//        RumbleControls rc = new RumbleControls(gamepad1);
//
//        Chassis chassis = VLRSubsystem.getInstance(Chassis.class);
//
//        if (runAuto) {
//            cs.schedule(new SequentialCommandGroup(
//                    new AutonomousPeriodActions(f),
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new RetractArm(),
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(300),
//                                            new FollowPath(f, bezierPath(BUCKET_HIGH_SCORE_POSE, SUB_PRE_BEZIER_POSE,
//                                                    new Pose(SUB_PRE_PREGRAB_POSE.getX(), SUB_PRE_PREGRAB_POSE.getY(), SUB_PRE_PREGRAB_POSE.getHeading()),
//                                                    new Pose(SUB_GRAB_POSE.getX(), SUB_GRAB_POSE.getY(), SUB_GRAB_POSE.getHeading()))
//                                                    .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), SUB_GRAB_POSE.getHeading())
//                                                    .build()).withTimeout(2500)
//                                    )
//                            ),
//                            new WaitCommand(1200),
//                            new SubmersibleGrab(f, alliance, reader)
//                    ),
//                    new ParallelCommandGroup(
//                            new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                            new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                            new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
//                            new FollowPath(f, bezierPath(new Pose(SUB_GRAB_POSE.getX(), SUB_GRAB_POSE.getY(), SUB_GRAB_POSE.getHeading()), SUB_PRE_BEZIER_POSE, BUCKET_HIGH_SCORE_TRUE_POSE)
//                                    .setLinearHeadingInterpolation(SUB_GRAB_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading())
//                                    .build()), // drive to the high basket
//                            new SequentialCommandGroup(
//                                    new SetCurrentArmState(ArmState.State.IN_ROBOT),
//                                    new WaitCommand(1600),
//                                    new SequentialCommandGroup(
//                                            new ScoreSample(110),
//                                            new WaitCommand(300),
//                                            new RetractArm()
//                                    )
//                            )
//                    )
//                    //new ScheduleNext(strategyController, f, alliance)
//            ));
//        }
//        final boolean[] followerActive = {true};
//
//        while (opModeIsActive()) {
//            gp.readButtons();
//
//            VLRSubsystem.getInstance(Wiper.class).wipe(gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//
//
//            if (gp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                cs.schedule(new SecondStageHangCommand(() -> gamepad1.dpad_down));
//            }
//        }
//    }
//}
