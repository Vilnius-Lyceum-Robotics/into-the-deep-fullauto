package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;

import java.util.logging.Level;

public class SetRotatorAngle extends ConditionalCommand {
    //TODO CONFIG FOR NEW HARDWARE VERSION
    private static final double cameraDangerMinAngle = 5;
    private static final double cameraDangerMaxAngle = 30;
    private static final double slideMaxSafeExtension = 0;


    public SetRotatorAngle(ArmRotatorConfiguration.TargetAngle angle) {
        this(angle.angleDegrees);
    }

    //SAFETY FEATURE NOT BONK THE LIMELIGHT WITH ARM BECAUSE OF HUMAN ERROR
    public SetRotatorAngle(double degrees) {
        super(
                new SequentialCommandGroup(
                        new LogCommand(ArmRotatorSubsystem.class, Level.WARNING, "YOU DUMB IDIOT, WHAT ARE YOU DOING!!!!"),
                        new WaitCommand(999999999)
                ),
                new InstantCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setTargetPosition(degrees)),
                () -> (isCameraInDanger(VLRSubsystem.getRotator().getAngleDegrees(), degrees))
        );
    }

    private static boolean isCameraInDanger(double a, double b) {
        return false;
        //return cameraDangerMinAngle <= Math.max(a, b) && cameraDangerMaxAngle >= Math.min(a, b) && VLRSubsystem.getSlides().getExtension() > slideMaxSafeExtension;
    }
}
