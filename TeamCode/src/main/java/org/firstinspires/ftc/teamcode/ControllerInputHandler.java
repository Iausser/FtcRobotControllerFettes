package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerInputHandler {
    private final Gamepad gamepad;

    public ControllerInputHandler(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean updateButton(Button button) {
        boolean hasToggled = false;
        boolean pressed = isButtonPressed(button.buttonType);

        if (!button.isPressed && pressed) {
            // toggle mode
            button.toggle();
            hasToggled = true;
        }

        button.isPressed = pressed;
        return hasToggled;
    }

    // Buttons
    public boolean isButtonPressed(String buttonName) {
        switch (buttonName) {
            case "options":
                return gamepad.options;
            case "cross":
                return gamepad.cross;
            case "square":
                return gamepad.square;
            case "circle":
                return gamepad.circle;
            case "triangle":
                return gamepad.triangle;
            case "dpadleft":
                return gamepad.dpad_left;
            case "dpadright":
                return gamepad.dpad_right;
            case "dpadup":
                return gamepad.dpad_up;
            case "dpaddown":
                return gamepad.dpad_down;
            case "lefttrigger":
                return gamepad.left_trigger > 0;
            case "righttrigger":
                return gamepad.right_trigger > 0;
            case "leftbumper":
                return gamepad.left_bumper;
            case "rightbumper":
                return gamepad.right_bumper;
            case "leftstickbutton":
                return gamepad.left_stick_button;
            case "rightstickbutton":
                return gamepad.right_stick_button;
            default:
                return false;
        }
    }

    // Other joystick and trigger methods for completeness
    public float getLeftStickX() {
        return gamepad.left_stick_x;
    }

    public float getLeftStickY() {
        return gamepad.left_stick_y;
    }

    public float getRightStickX() {
        return gamepad.right_stick_x;
    }

    public float getRightStickY() {
        return gamepad.right_stick_y;
    }

    public float leftTrigger() {
        return gamepad.left_trigger;
    }

    public float rightTrigger() {
        return gamepad.right_trigger;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    public boolean rightBumper() {
        return gamepad.right_bumper;
    }
}
