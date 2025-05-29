package org.firstinspires.ftc.teamcode.helpers.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.RED;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.YELLOW;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.monitoring.GlobalLoopTimeMonitor;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalTimer;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.persistence.AllianceSaver;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.BlinkinSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.ArrayList;
import java.util.List;


public abstract class VLRAutoTestOpMode extends VLRLinearOpMode {
    CommandScheduler cs;
    Follower f;
    Command autoCommand = new InstantCommand();
    AutoConfigurator ac;

    @Override
    public void run() {
        cs = CommandScheduler.getInstance();

        FConstants.initialize();
        f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(StartPose());

        //noinspection unchecked
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, BlinkinSubsystem.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        LimelightYoloReader reader = new LimelightYoloReader();

        autoCommand = autoCommand(f, reader);

        ///SCHEDULE AUTO COMMAND, THEN SAVE PEDRO POSE AND LOG TOTAL AUTO TIME
        cs.schedule(new SequentialCommandGroup(
                autoCommand,
                new ScheduleRuntimeCommand(() -> new InstantCommand(() -> PoseSaver.setPedroPose(f.getPose()))),
                GlobalTimer.logAutoTime()
        ));

        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(ClawConfiguration.VerticalRotation.UP);
        VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(ClawConfiguration.HorizontalRotation.NORMAL);
        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetState(ClawConfiguration.GripperState.CLOSED_LOOSE);
        VLRSubsystem.getInstance(BlinkinSubsystem.class).setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);
        GlobalLoopTimeMonitor.reset();

        ac = new AutoConfigurator(telemetry, gamepad1, ()-> isStopRequested() || opModeIsActive());
        AutoConfigurator.Choice color = ac.multipleChoice("Select alliance:", new AutoConfigurator.Choice("Blue"),
                new AutoConfigurator.Choice("Red"));

        ac.review("Selected alliance: " + color.text);


        if (!isStopRequested()) {
            boolean isBlue = color.text.equals("Blue");
            List<LimelightYoloReader.Limelight.Sample.Color> allowedColors = new ArrayList<>();

            if (isBlue) {allowedColors.add(BLUE);}
            else {allowedColors.add(RED);}
            if (!SpecimenOnly()) {allowedColors.add(YELLOW);}

            reader.setAllowedColors(allowedColors);
            AllianceSaver.setAlliance(isBlue ? Alliance.BLUE : Alliance.RED);

            waitForStart();
            GlobalTimer.resetTimer();

            GlobalConfig.DEBUG_MODE = false;
            Init();

            while (opModeInInit()) {
                InitLoop();
                super.idle();
            }
            Start();
        }


        while (opModeIsActive()) {
            long startTime = System.nanoTime();
            Loop();
            f.update();

            GlobalLoopTimeMonitor.setMainLoopDurationNs(System.nanoTime() - startTime);
        }
        Stop();
    }

    public void Loop() {
    }

    public void Init() {
    }

    public void Start() {
    }

    public void Stop() {
    }

    public void InitLoop() {
    }

    public abstract Command autoCommand(Follower f, LimelightYoloReader reader);

    public abstract Pose StartPose();

    public abstract boolean SpecimenOnly();
}