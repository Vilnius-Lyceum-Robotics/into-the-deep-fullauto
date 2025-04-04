package org.firstinspires.ftc.teamcode.subsystems.neopixel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
@Config
@TeleOp(name = "AAAA Test NeoPixel", group = "Test")
public class TestNeoPixelOpMode extends VLRLinearOpMode {

    NeoPixelSubsystem neoPixelSubsystem;
    public static NeoPixelConfiguration.Colour colour = NeoPixelConfiguration.Colour.RED;
    public static NeoPixelConfiguration.Effect effect = NeoPixelConfiguration.Effect.SOLID_COLOR;
    public static double effectTime = 1;
    public static double brightness = 1;

    public static int trailLength = 4;
    @Override
    public void run() {
        // Initialize the I2C device
        VLRSubsystem.requireSubsystems(NeoPixelSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);
        neoPixelSubsystem = VLRSubsystem.getInstance(NeoPixelSubsystem.class);

        telemetry.addData("Status", "Initialized NeoPixel Driver");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            neoPixelSubsystem.setColor(colour);
            neoPixelSubsystem.setEffect(effect);
            neoPixelSubsystem.setEffectTime(effectTime);
            neoPixelSubsystem.setBrightness(brightness);
            neoPixelSubsystem.setTrailLength(trailLength);


        }

        if(isStopRequested()){
            neoPixelSubsystem.clear();
            neoPixelSubsystem.periodic();
        }

    }
}
