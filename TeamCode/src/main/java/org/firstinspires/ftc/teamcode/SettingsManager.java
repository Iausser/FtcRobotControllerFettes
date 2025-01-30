package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SettingsManager {
    public ControllerInputHandler controllerInput;
    public Gamepad gamepad;
    public Telemetry telemetry;
    public RobotMove robotMove;
    public Button settingsButton;

    public SettingsManager(Gamepad gamepad, RobotMove robotMove, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.robotMove = robotMove;
        this.telemetry = telemetry;
        controllerInput = new ControllerInputHandler(gamepad);

        // initialise buttons
        settingsButton = new Button("options", false);
    }


    public void printSettings() {
        telemetry.clearAll();
        telemetry.addData("", "Options:");
        telemetry.addData("", "Press Square to toggle the movement mode");
        telemetry.addData("", "Press Cross to set a new default orientation");
        telemetry.addData("", "\n");
    }

    public void doSettings() {
        if (controllerInput.updateButton(robotMove.fieldCentricMovement) && robotMove.fieldCentricMovement.onMode) {
            // toggle movement mode
            telemetry.clearAll();
            telemetry.addData("", "Switched to Field Centric Movement");
        }

        if (controllerInput.updateButton(robotMove.robotCentricMovement) && robotMove.robotCentricMovement.onMode) {
            // toggle movement mode
            telemetry.clearAll();
            telemetry.addData("", "Switched to Robot Centric Movement");
        }

        if (controllerInput.updateButton(robotMove.orientationButton)) {
            // set new default orientation
            robotMove.setDefaultOrientation();
            telemetry.clearAll();
            telemetry.addData("", "New default orientation set");
        }
    }
}

