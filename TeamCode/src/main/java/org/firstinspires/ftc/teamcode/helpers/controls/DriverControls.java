package org.firstinspires.ftc.teamcode.helpers.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * Abstraction for gamepad controls.
 * Defines controls using ControlDefinitions instead of actually writing if statements or similar.
 */
public class DriverControls {
    /**
     * The extended gamepad object.
     */
    GamepadEx gamepad;

    /**
     * List of control definitions.
     */
    List<ControlDefinition> controls = new java.util.ArrayList<>();

    /**
     * Handler for left stick input.
     */
    BiConsumer<Double, Double> leftStickHandler;

    /**
     * Handler for right stick input.
     */
    BiConsumer<Double, Double> rightStickHandler;

    /**
     * Handler for both sticks input.
     */
    QuadConsumer<Double, Double, Double, Double> bothSticksHandler;

    /**
     * Handler for vibrations
     */
    Supplier<Boolean> vibration;

    protected final GamepadKeys.Button TRIANGLE = GamepadKeys.Button.Y;
    protected final GamepadKeys.Button SQUARE = GamepadKeys.Button.X;
    protected final GamepadKeys.Button CROSS = GamepadKeys.Button.A;
    protected final GamepadKeys.Button CIRCLE = GamepadKeys.Button.B;

    /**
     * Constructs a DriverControls object.
     *
     * @param gamepad The extended gamepad object to use for controls.
     */
    public DriverControls(GamepadEx gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Adds a control definition to the list of controls.
     *
     * @param control The control definition to add.
     */
    public void add(ControlDefinition control) {
        this.controls.add(control);
    }

    /**
     * Sets the handler for left stick input.
     *
     * @param control The BiConsumer to handle left stick input (y, x).
     */
    public void addLeftStickHandler(BiConsumer<Double, Double> control) {
        leftStickHandler = control;
    }

    /**
     * Sets the handler for right stick input.
     *
     * @param control The BiConsumer to handle right stick input (y, x).
     */
    public void addRightStickHandler(BiConsumer<Double, Double> control) {
        rightStickHandler = control;
    }

    public void addBothSticksHandler(QuadConsumer<Double, Double, Double, Double> control) {
        bothSticksHandler = control;
    }

    public void addVibration(Supplier<Boolean> supplier) {
        vibration = supplier;
    }

    /**
     * Updates all controls and executes their associated actions.
     * This method should be called in the main control loop.
     */
    public void update() {
        if (bothSticksHandler != null)
            bothSticksHandler.accept(gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightY(), gamepad.getRightX());
        else {
            if (leftStickHandler != null)
                leftStickHandler.accept(gamepad.getLeftY(), gamepad.getLeftX());
            if (rightStickHandler != null)
                rightStickHandler.accept(gamepad.getRightY(), gamepad.getRightX());
        }

        for (ControlDefinition control : controls) {
            control.run(gamepad);
        }
    }
}