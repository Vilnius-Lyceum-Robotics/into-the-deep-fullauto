package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

@TeleOp(name="Limelight YOLO output", group = "Utils")
public class LimelightVerification extends VLRLinearOpMode {

    @Override
    public void run() {
        LimelightYoloReader reader = new LimelightYoloReader();

        //VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("JSON output", reader.getRawJSONRequest("/detection_boxes.json"));
            reader.getRawJSONRequest("/request_frame");
//            if (reader.getBestSample() != null) {
//                VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(Math.toDegrees(reader.getBestSample().getAngle()) / 180);
//            }
            telemetry.update();
            sleep(500);
        }
    }
}
