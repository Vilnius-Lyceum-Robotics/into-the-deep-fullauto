package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.IntakeSample;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetColour;

@Config
public class GrabBucketSample extends SequentialCommandGroup {
    public static double SLIDE = 0.2;

    public GrabBucketSample(boolean clawTwisted) {
        addCommands(
                new CustomConditionalCommand(
                        new SequentialCommandGroup(
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(100),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitCommand(150)
                        ),
                        () -> clawTwisted
                ),
                new IntakeSample(SLIDE),
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new WaitCommand(100),
                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                new WaitCommand(120),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //new SetColour(NeoPixelConfiguration.Colour.CYAN),

                new WaitCommand(200),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetCurrentArmState(ArmState.State.IN_ROBOT)
        );
    }

    public GrabBucketSample() {
        this(false);
    }
}