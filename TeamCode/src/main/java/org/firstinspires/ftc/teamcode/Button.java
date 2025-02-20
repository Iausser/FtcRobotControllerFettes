package org.firstinspires.ftc.teamcode;

public class Button {
    public final String buttonType;
    public boolean onMode;     // Indicates whether the button is in 'on' mode
    public boolean isPressed;  // Indicates whether the button is currently pressed

    // Constructor
    public Button(String buttonType, boolean initialMode) {
        this.buttonType = buttonType;
        this.onMode = initialMode;
        this.isPressed = false;
    }

    // Method to toggle the onMode of the button
    public void toggle() {
        onMode = !onMode;  // Flip the current state of onMode
    }

    public boolean isOn() {
        return onMode;
    }
}
