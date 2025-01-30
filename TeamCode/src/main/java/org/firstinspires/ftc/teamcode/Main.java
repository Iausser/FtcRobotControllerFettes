package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {
    private ControllerInputHandler controllerInput;
    private RobotMove robotMove;
    private RobotArm robotArm;
    private SettingsManager settings;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap, gamepad1, telemetry);
        robotArm = new RobotArm(hardwareMap, gamepad1, telemetry);
        settings = new SettingsManager(gamepad1, robotMove, telemetry);

        robotMove.setDefaultOrientation();
    }

    @Override
    public void loop() {
        manageButtons();

        if (settings.settingsButton.onMode) {
            telemetry.addData("Settings Mode Active", "Stopping robot movement");
            robotMove.robotCentricMovement(0, 0, 0, 0); // Ensure all motors are stopped
            settings.doSettings();
        } else {
            telemetry.addData("Left Stick X", controllerInput.getLeftStickX());
            telemetry.addData("Left Stick Y", controllerInput.getLeftStickY());
            telemetry.addData("Right Stick X", controllerInput.getRightStickX());

            robotMove.doRobotMovement();
            robotArm.doArmMovement(); // Execute arm movement logic
            feedbackPositions();
        }
        telemetry.update();
    }

    private void feedbackPositions() {
        telemetry.addData("\nIMU orientation:", robotMove.getIMUOrientation().firstAngle);
        telemetry.addData("Auto correct orientation:", robotMove.autoCorrectOrientation.firstAngle);
        telemetry.update();
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
