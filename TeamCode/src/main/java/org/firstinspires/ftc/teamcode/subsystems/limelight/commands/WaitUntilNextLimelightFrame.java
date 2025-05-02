package org.firstinspires.ftc.teamcode.subsystems.limelight.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

public class WaitUntilNextLimelightFrame extends CommandBase {

    ElapsedTime elapsedTime = new ElapsedTime();
    LimelightYoloReader reader;
    boolean started = false;

    public WaitUntilNextLimelightFrame(LimelightYoloReader reader) {
        this.reader = reader;
    }

    @Override
    public void initialize() {
        elapsedTime.reset();
        started = true;
    }

    @Override
    public void execute() {
        System.out.println("WAIT UNTIL NEXT LIMELIGHT FRAME COMMAND, ELAPSED TIME: " + elapsedTime.seconds() + "s, DELTA: " + reader.getFrameTimeDelta());
    }

    @Override
    public boolean isFinished() {
        if (!started) return false;

        if (elapsedTime.milliseconds() > 1000) return true;

        if (reader.getFrameTimeDelta() == -1) return false;

        return (elapsedTime.milliseconds() + 100.0) > reader.getFrameTimeDelta();
    }
}
