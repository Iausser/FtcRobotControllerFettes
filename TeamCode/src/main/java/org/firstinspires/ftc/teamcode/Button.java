package org.firstinspires.ftc.teamcode;

public class Button {
    String buttonType;
    boolean onMode;
    boolean isPressed;

    public Button(String buttonType, boolean initialMode) {
        this.buttonType = buttonType;
        onMode = initialMode;
        isPressed = false;
    }
}
