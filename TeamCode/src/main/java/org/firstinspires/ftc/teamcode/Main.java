package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends OpMode {
    private ControllerInputHandler controllerInput;
    private RobotMove robotMove;
    private SettingsManager settings;
    private RobotArm robotArm;
    private RobotExtras robotExtras;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap, gamepad1, telemetry);
        robotArm = new RobotArm(hardwareMap, gamepad1, telemetry);
        robotExtras = new RobotExtras(hardwareMap, gamepad1, telemetry);
        settings = new SettingsManager(gamepad1, robotMove, robotExtras, telemetry);
    }

    @Override
    public void loop() {
        manageButtons();

        if (settings.settingsButton.onMode) {
            robotMove.robotCentricMovement(0, 0, 0, 0);
            settings.doSettings();
        } else {
            robotMove.doRobotMovement();
            robotArm.doArmMovement();
            robotExtras.doHardwareMovement();
            feedbackPositions();
        }
        telemetry.update();
    }

    private void feedbackPositions() {
        telemetry.addData("Motor arm left:", robotArm.motorArmLeft.getCurrentPosition());
        telemetry.addData("Motor arm right:", robotArm.motorArmLeft.getCurrentPosition());
        telemetry.addData("Servo arm:", robotArm.servoArmAngle);
        telemetry.addData("Servo hand:", robotArm.servoHand.getPosition());

        telemetry.addData("\nIMU orientation:", robotMove.getIMUOrientation().firstAngle);
        telemetry.addData("Auto correct orientation:", robotMove.autoCorrectOrientation.firstAngle);
        telemetry.update();
    }

    private void manageButtons() {
        // check settings button
        if (controllerInput.updateButton(settings.settingsButton)) {
            if (settings.settingsButton.onMode) {
                // in settings, print commands
                settings.printSettings();
            } else {
                // exited settings mode
                telemetry.clearAll();
            }
        }
    }
}
