package org.firstinspires.ftc.teamcode.helpers.testOpmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wiper.Wiper;

@TeleOp(name = "Sensor Subsystem Test", group = "Utils")
public class SensorsTest extends VLRLinearOpMode {

    /*@Override
    public void init() {
        sensors = VLRSubsystem.getInstance(SensorsSubsystem.class);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        sensors.updateBackDistance();
        sensors.updateLeftDistance();
        sensors.updateRightDistance();

        telemetry.addData("LEFT DISTANCE: ", sensors.getBackDistance());
        telemetry.addData("RIGHT DISTANCE: ", sensors.getRightDistance());
        telemetry.addData("BACK DISTANCE: ", sensors.getBackDistance());
    }*/

    @Override
    public void run(){
        VLRSubsystem.requireSubsystems(SensorsSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);


        waitForStart();



        while(opModeIsActive()){
            telemetry.addData("LEFT DISTANCE: ", VLRSubsystem.getInstance(SensorsSubsystem.class).getLeftDistance());
            telemetry.addData("RIGHT DISTANCE: ", VLRSubsystem.getInstance(SensorsSubsystem.class).getRightDistance());
            telemetry.addData("BACK DISTANCE: ", VLRSubsystem.getInstance(SensorsSubsystem.class).getBackDistance());


            telemetry.update();
            sleep(100);
        }
    }

}
