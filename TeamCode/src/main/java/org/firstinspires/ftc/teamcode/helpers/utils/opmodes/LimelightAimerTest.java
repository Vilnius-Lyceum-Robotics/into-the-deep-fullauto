package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.SUB_GRAB;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrab;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

@TeleOp(name = "LimelightAimer", group = "Utils")
@Photon
@Config
public class LimelightAimerTest extends VLRLinearOpMode {

    public static boolean go = false;

    @Override
    public void run() {

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        Follower f  = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(SUB_GRAB);
        CommandScheduler cs = CommandScheduler.getInstance();
        LimelightYoloReader reader = new LimelightYoloReader();
        VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(0.6);


        waitForStart();

        while (!go) {
            sleep(10);
        }

        cs.schedule(new SequentialCommandGroup(
                new SubmersibleGrab(f, Alliance.BLUE, reader),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetArmPosition().extension(0)
        ));

        while (opModeIsActive()) {
            f.update();
        }
    }
}