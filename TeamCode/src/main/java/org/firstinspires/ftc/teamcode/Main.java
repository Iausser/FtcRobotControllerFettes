package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {
    private ControllerInputHandler controllerInput;
    private RobotMove robotMove;
    private RobotExtras robotExtras;
    private SettingsManager settings;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap, gamepad1, telemetry);
        robotExtras = new RobotExtras(hardwareMap, gamepad1, telemetry);
        settings = new SettingsManager(gamepad1, robotMove, telemetry);
    }

    @Override
    public void loop() {
        manageButtons();
        if (settings.settingsButton.onMode) {
            telemetry.clearAll();
            telemetry.addData("Settings Mode Active", "Stopping robot movement\n");
            robotMove.robotCentricMovement(0, 0, 0, 0); // Ensure all motors are stopped
            settings.printSettings();
            settings.doSettings();
        } else {
            telemetry.addData("Left Stick X", controllerInput.getLeftStickX());
            telemetry.addData("Left Stick Y", controllerInput.getLeftStickY());
            telemetry.addData("Right Stick X", controllerInput.getRightStickX());

            robotMove.doRobotMovement();
            robotExtras.doHardwareMovement();  // Execute pulley, arm and hand movement logic
            feedbackPositions();
        }
        telemetry.update();
    }

    private void feedbackPositions() {
        telemetry.addData("\nIMU orientation:", robotMove.getIMUOrientation().firstAngle);
        telemetry.addData("Auto correct orientation:", robotMove.autoCorrectOrientation.firstAngle);
    }

    private void manageButtons() {
        if (controllerInput.updateButton(settings.settingsButton)) {
            if (settings.settingsButton.onMode) {
                settings.printSettings();
            } else {
                telemetry.clearAll();
            }
        }
    }
}