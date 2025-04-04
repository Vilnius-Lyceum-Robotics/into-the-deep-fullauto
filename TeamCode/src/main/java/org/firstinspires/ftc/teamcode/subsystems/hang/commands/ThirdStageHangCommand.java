package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmState.State.HANG_SECOND_STAGE;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmOperationMode;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetColour;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetEffect;

import java.util.function.BooleanSupplier;

public class ThirdStageHangCommand extends SequentialCommandGroup {
    public ThirdStageHangCommand(BooleanSupplier gamepadCondition, BooleanSupplier interruptCondition) {
        addRequirements(VLRSubsystem.getRotator(), VLRSubsystem.getSlides(), VLRSubsystem.getHang());
        addCommands(
                new CustomConditionalCommand(
                        new RetractArm(),
                        () -> !ArmState.isCurrentState(ArmState.State.IN_ROBOT, ArmState.State.HANG_THIRD_STAGE, HANG_SECOND_STAGE)
                ),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),

                new InstantCommand(()-> VLRSubsystem.getHang().setTargetAngleUP()),
                new SetRotatorAngle(102.5),
                new WaitUntilCommand(()-> VLRSubsystem.getRotator().getAngleDegrees() >= 80),
                new SetSlideExtension(0.314),
                new WaitUntilCommand(()-> (VLRSubsystem.getRotator().reachedTargetPosition() && VLRSubsystem.getSlides().reachedTargetPosition())).withTimeout(2000),
                new WaitCommand(500),

                new WaitUntilCommand(gamepadCondition),
                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.HANG_SLOW),

                new SetSlideExtension(0.15),
                new WaitCommand(100),
                new SetRotatorAngle(85),
                new WaitCommand(300),
                new InstantCommand(()-> VLRSubsystem.getHang().setPower(0.2)),
                new WaitUntilCommand(()->VLRSubsystem.getHang().analogFeedbackThresholdReached()),
                new SetSlideExtension(0.3),
                new InstantCommand(()-> VLRSubsystem.getHang().setPower(0)),

                new WaitCommand(10000000),


                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.NORMAL),

                new SetSlideExtension(0.3),
                new InstantCommand(()-> VLRSubsystem.getInstance(HangSubsystem.class).setPower(0)),


                new SetSlideExtension(0.888),
                new WaitCommand(100),
                new SetRotatorAngle(80),

                new WaitUntilCommand(()-> (VLRSubsystem.getRotator().reachedTargetPosition() && VLRSubsystem.getSlides().reachedTargetPosition())).withTimeout(2000),
                new InstantCommand(()-> VLRSubsystem.getRotator().setMappedCoefficients()),

                new SetRotatorAngle(102),
                new WaitCommand(360),

                new SetSlideExtension(0.83),
                new WaitCommand(300),
                new SetRotatorAngle(99),


                new WaitUntilCommand(gamepadCondition),



                new SetArmOperationMode(ArmSlideConfiguration.OperationMode.HANG_FAST),
//                new InstantCommand(()-> VLRSubsystem.getSlides().setPowerOverride(true)),
//                new InstantCommand(()-> VLRSubsystem.getSlides().setMotorPower(0)),
                //new WaitCommand(500),

                //new InstantCommand(()-> VLRSubsystem.getSlides().setMotorPower(-1)),


                new SetSlideExtension(0.04),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().getExtension() < 0.79),

                //new InstantCommand(()-> VLRSubsystem.getRotator().deactivateRotatorForHang()),

                //new SetRotatorAngle(130),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().getExtension() < 0.28),
                new SetHangPosition(HangConfiguration.TargetPosition.DOWN),

                new SetRotatorAngle(35),
                new WaitCommand(200),
                //new InstantCommand(()-> VLRSubsystem.getRotator().reenableMotorForHang()),
                new WaitUntilCommand(() -> VLRSubsystem.getSlides().reachedTargetPosition()).withTimeout(3000),



                new WaitUntilCommand(gamepadCondition),
                new SetRotatorAngle(120)
        );
    }


    public ThirdStageHangCommand(BooleanSupplier gamepadCondition){
        this(gamepadCondition, ()-> false);
    }
}