package org.firstinspires.ftc.teamcode.subsystems.limelight.commands;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

public class RequestLimelightFrame extends InstantCommand {
    LimelightYoloReader reader;

    public RequestLimelightFrame(LimelightYoloReader reader) {
        this.reader = reader;
    }
    @Override
    public void run() {
        reader.requestFrame();
    }
}
