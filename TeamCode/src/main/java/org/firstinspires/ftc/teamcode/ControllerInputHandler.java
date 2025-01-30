package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerInputHandler {
    private Gamepad gamepad;
    private boolean isFineMovement = false;  // Toggle flag for fine movement mode
    private double speedMultiplier = 1.0;    // Default to full speed
    private boolean previousLeftStickButton = false;  // Track left stick button state

    public ControllerInputHandler(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // Toggle fine movement mode
    private void toggleFineMovement() {
        isFineMovement = !isFineMovement;
        speedMultiplier = isFineMovement ? 0.5 : 1.0;  // Adjust speed multiplier for fine movement
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public boolean updateButton(Button button) {
        boolean currentState = isButtonPressed(button.getName());

        // Only toggle if the button state has changed from not pressed to pressed
        if (currentState && !button.isPressed()) {
            button.toggle();
            return true;  // Return true to indicate the button was toggled
        }

        button.setPressed(currentState);  // Update the button's current state
        return false;  // Return false if there was no toggle
    }

    public void handleJoystickInput(RobotMove robotDrive) {
        // Toggle fine movement mode only on state change of left stick button
        if (gamepad.left_stick_button && !previousLeftStickButton) {
            toggleFineMovement();
        }
        previousLeftStickButton = gamepad.left_stick_button;

        // Call the doRobotMovement method in RobotMove to handle joystick input
        robotDrive.doRobotMovement();
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
        return -gamepad.left_stick_y;
    }

    public float getRightStickX() {
        return gamepad.right_stick_x;
    }

    public float getRightStickY() {
        return -gamepad.right_stick_y;
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
