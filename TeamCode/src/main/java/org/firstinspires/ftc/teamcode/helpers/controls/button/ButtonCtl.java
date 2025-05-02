package org.firstinspires.ftc.teamcode.helpers.controls.button;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.helpers.controls.ControlDefinition;

import java.util.function.Consumer;

/**
 * Implements a button control definition.
 * This class defines how a specific button on the gamepad should behave and what action it should trigger.
 */
public class ButtonCtl implements ControlDefinition {
    private final GamepadKeys.Button button;
    private final Trigger trigger;
    private final boolean returnOnlyOnTrue;
    private final Consumer<Boolean> action;

    private ButtonReader reader;

    /**
     * Defines the types of button triggers.
     */
    public enum Trigger {
        SIMPLE,
        WAS_JUST_RELEASED,
        WAS_JUST_PRESSED,
        STATE_JUST_CHANGED
    }

    /**
     * Constructs a ButtonCtl with specified parameters.
     *
     * @param button           The button to be controlled.
     * @param trigger          The type of trigger for the button.
     * @param returnOnlyOnTrue Whether to execute the action only when the trigger condition is true.
     * @param action           The action to be performed when the button is triggered.
     */
    public ButtonCtl(GamepadKeys.Button button, Trigger trigger, boolean returnOnlyOnTrue, Consumer<Boolean> action) {
        this.button = button;
        this.trigger = trigger;
        this.returnOnlyOnTrue = returnOnlyOnTrue;
        this.action = action;
    }

    /**
     * Constructs a ButtonCtl with default returnOnlyOnTrue set to false.
     *
     * @param button  The button to be controlled.
     * @param trigger The type of trigger for the button.
     * @param action  The action to be performed when the button is triggered.
     */
    public ButtonCtl(GamepadKeys.Button button, Trigger trigger, Consumer<Boolean> action) {
        this(button, trigger, true, action);
    }

    /**
     * Executes the button control based on the current gamepad state.
     *
     * @param gamepad The FTCLib GamepadEx object containing the current input state.
     */
    @Override
    public void run(GamepadEx gamepad) {
        boolean result;
        if (reader == null) {
            GamepadButton btn = gamepad.getGamepadButton(button);
            reader = new ButtonReader(btn::get);
        }
        reader.readValue();
        switch (trigger) {
            case SIMPLE:
                result = reader.isDown();
                break;
            case WAS_JUST_RELEASED:
                result = reader.wasJustReleased();
                break;
            case WAS_JUST_PRESSED:
                result = reader.wasJustPressed();
                break;
            case STATE_JUST_CHANGED:
                result = reader.stateJustChanged();
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + trigger);
        }

        if (!returnOnlyOnTrue || result) action.accept(result);
    }
}
