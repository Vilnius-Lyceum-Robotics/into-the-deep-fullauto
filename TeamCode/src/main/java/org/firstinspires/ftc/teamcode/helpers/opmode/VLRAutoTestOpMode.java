package org.firstinspires.ftc.teamcode.helpers.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader.Limelight.Sample.Color.YELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.helpers.persistence.PoseSaver;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.Arrays;

import pedroPathing.tuners.constants.FConstants;
import pedroPathing.tuners.constants.LConstants;

public abstract class VLRAutoTestOpMode extends VLRLinearOpMode {
    CommandScheduler cs;
    Follower f;
    ElapsedTime autoTimer = new ElapsedTime();

    Command autoCommand;
    boolean autoFinished = false;
    boolean prevAutoFinished = false;
    double autoTime = 0;

    @Override
    public void run() {
        cs = CommandScheduler.getInstance();
        FConstants.initialize();

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LimelightYoloReader reader = new LimelightYoloReader();
        reader.setAllowedColors(Arrays.asList(BLUE, YELLOW));

        f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(StartPose());
        beforeEndRunnable = () -> PoseSaver.setPedroPose(f.getPose());

        autoCommand = autoCommand(f, reader);
        cs.schedule(autoCommand.andThen(new InstantCommand(() -> autoFinished = true)));

        waitForStart();
        autoTimer.reset();

        GlobalConfig.DEBUG_MODE = false;

        Init();
        while (opModeInInit()) {
            InitLoop();
        }

        waitForStart();

        Start();
        while (opModeIsActive()) {
            Loop();
            f.update();

            if (autoFinished) {
                if (!prevAutoFinished) {
                    prevAutoFinished = true;
                    autoTime = autoTimer.seconds();
                }
                System.out.println("TOTAL AUTO TIME: " + autoTime);
            }
        }
        Stop();
    }

    public void Loop() {
    }

    ;

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
}
