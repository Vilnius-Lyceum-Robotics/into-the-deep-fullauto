package org.firstinspires.ftc.teamcode.helpers.utils.opmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRTestOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;

@Config
@Photon
@TeleOp()
public class ArmCommandMagnitudeAndExtensionTest extends VLRTestOpMode {
    public static double extension = 0;
    public static double angle = 0;

    private double prevExtension = 0;
    private double prevAngle = 0;

    boolean prevTriangle = false;
    boolean prevCross = false;
    boolean prevSquare = false;
    boolean prevCircle = false;

    Follower follower;
    GamepadEx gamepad;
    RumbleControls rc;


    @Override
    public void Start(){
        //CommandScheduler.getInstance().schedule(new SetArmPosition().X(20, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
    }

    @Override
    public void Init(){
        FConstants.initialize();

        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose());

        gamepad = new GamepadEx(gamepad1);
        rc = new RumbleControls(gamepad1);
    }

    @Override
    public void Loop() {
//        if (extension != prevExtension){
//            prevExtension = extension;
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(extension));
//        }
//
//        if (angle != prevAngle){
//            prevAngle = angle;
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(angle));
//        }

        if (gamepad1.triangle && !prevTriangle){
            CommandScheduler.getInstance().schedule(new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET));
        }

        else if (gamepad1.cross && !prevCross){
            CommandScheduler.getInstance().schedule(new SetArmPosition().retract());
        }

        else if (gamepad1.square && !prevSquare){
            CommandScheduler.getInstance().schedule(new SetArmPosition().intakeSample(0.34));
        }

//        if (gamepad1.triangle && !prevTriangle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(1));
//        }
//
//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().extension(0));
//        }
//
//        else if (gamepad1.square && !prevSquare){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(90));
//        }
//
//        else if (gamepad1.circle && !prevCircle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(0));
//        }


//        if (gamepad1.triangle && !prevTriangle){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().XY(40, 30, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
//        }
//
//        else if (gamepad1.cross && !prevCross){
//            CommandScheduler.getInstance().schedule(new SetArmPosition().XY(10, 0, MainArmConfiguration.OFFSET_REFERENCE_PLANE.FRONT));
//        }


        prevTriangle = gamepad1.triangle;
        prevCross = gamepad1.cross;
        prevSquare = gamepad1.square;
        prevCircle = gamepad1.circle;


        follower.update();
        VLRSubsystem.getInstance(Chassis.class).drive(0, 0, 0);

        telemetry.update();
    }
}