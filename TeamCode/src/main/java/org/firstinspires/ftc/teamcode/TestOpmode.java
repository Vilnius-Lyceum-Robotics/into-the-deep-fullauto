package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.frozenmilk.sinister.loading.Pinned;

@Config
@TeleOp
public class TestOpmode extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        System.out.println(11111);
    }
}
