package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;

public class SetArmOperationMode extends SequentialCommandGroup {
    public SetArmOperationMode(ArmSlideConfiguration.OperationMode operationMode){
        addCommands(
                new InstantCommand(()-> VLRSubsystem.getSlides().setOperationMode(operationMode)),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> VLRSubsystem.getRotator().setHangCoefficients()),
                                new InstantCommand(()-> VLRSubsystem.getSlides().setHangCoefficients())
                        ),
                        ()-> operationMode == ArmSlideConfiguration.OperationMode.HANG_SLOW
                ),

                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> VLRSubsystem.getRotator().setHangCoefficients()),
                                new InstantCommand(()-> VLRSubsystem.getSlides().setHangCoefficientsFast())
                        ),
                        ()-> operationMode == ArmSlideConfiguration.OperationMode.HANG_FAST
        )

        );
    }
}
