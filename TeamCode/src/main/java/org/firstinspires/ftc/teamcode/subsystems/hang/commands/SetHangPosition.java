package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;



public class SetHangPosition extends CommandBase {
    private HangSubsystem hang;
    private HangConfiguration.TargetPosition targetPosition;
    private ElapsedTime timer;
    private boolean hasStarted = false;

    public SetHangPosition(HangConfiguration.TargetPosition targetPosition){
        addRequirements(VLRSubsystem.getInstance(HangSubsystem.class));

        hang = VLRSubsystem.getInstance(HangSubsystem.class);
        this.targetPosition = targetPosition;
    }


    @Override
    public void initialize(){
        timer = new ElapsedTime();

        switch (targetPosition){
            case DOWN:
                hang.setPower(-0.4);
                break;
            case UP:
                hang.setPower(0.75);
                break;
        }
    }

    @Override
    public void execute(){
        if (!hasStarted){
            timer.reset();
            hasStarted = true;
        }
    }


    @Override
    public void end(boolean interrupted){
            switch (targetPosition) {
                case UP:
                    hang.setPower(0);
                    break;
                case DOWN:
                    hang.setPower(0);
                    break;
            }
    }


    @Override
    public boolean isFinished() {
        return timer.seconds() > 0.4;
    }
}
