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
            // in settings mode
            telemetry.addData("\nSettings Mode Active", "Stopping robot movement\n");
            robotMove.robotCentricMovement(0, 0, 0, 0); // Ensure all motors are stopped
            settings.doSettings();
        } else {
            // in movement mode
            robotMove.doRobotMovement();
            robotExtras.doHardwareMovement();  // Execute pulley, arm and hand movement logic
            telemetry.clearAll();
            feedbackValues();
        }
        telemetry.update();
    }

    private void feedbackValues() {
        telemetry.addData("Left Stick X", controllerInput.getLeftStickX());
        telemetry.addData("Left Stick Y", controllerInput.getLeftStickY());
        telemetry.addData("Right Stick X", controllerInput.getRightStickX());

        telemetry.addData("\nIMU orientation:", robotMove.getIMUOrientation().firstAngle);
        telemetry.addData("Auto correct orientation:", robotMove.autoCorrectOrientation.firstAngle);

        telemetry.addData("\nHand angle:", robotExtras.getHandAngle());
    }

    private void manageButtons() {
        if (controllerInput.updateButton(settings.settingsButton) && settings.settingsButton.onMode) {
            settings.printSettings();
        }
    }
}