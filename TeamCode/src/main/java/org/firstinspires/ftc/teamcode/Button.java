package org.firstinspires.ftc.teamcode;

public class Button {
    private String buttonType;
    public boolean onMode;     // Indicates whether the button is in 'on' mode
    private boolean isPressed;  // Indicates whether the button is currently pressed
    private boolean lastPressedState;  // Track the previous state to detect button press

    // Constructor
    public Button(String buttonType, boolean initialMode) {
        this.buttonType = buttonType;
        this.onMode = initialMode;
        this.isPressed = false;
        this.lastPressedState = false;
    }

    // Method to toggle the onMode of the button
    public void toggle() {
        onMode = !onMode;  // Flip the current state of onMode
    }

    // Method to get the button's name/type
    public String getName() {
        return buttonType;
    }

    // Method to check if the button is currently on
    public boolean isOn() {
        return onMode;
    }

    // Method to update the button's press state (for example, from input)
    public void setPressed(boolean isPressed) {
        this.isPressed = isPressed;
    }

    // Method to check if the button is pressed (detection of state change)
    public boolean isPressed() {
        return isPressed;
    }

    // Method to update button state based on whether it was just pressed or released
    public void update() {
        // Detect button press (when it was released last frame and pressed now)
        if (!lastPressedState && isPressed) {
            toggle();  // Toggle the state
        }
        lastPressedState = isPressed;  // Update the last pressed state
    }
}
