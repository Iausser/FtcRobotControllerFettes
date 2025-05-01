package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorTest", group = "TeleOp")
public class MotorTest extends OpMode {

    private RobotMove robotMove;
    private ControllerInputHandler controllerInput;
    private static final double MOTOR_POWER = 0.2; // Define MOTOR_POWER

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap, gamepad1, telemetry);

        // Reset the IMU's default orientation
        //robotMove.setDefaultOrientation();
    }

    @Override
    public void loop() {
        // Debugging gamepad inputs
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);

        // Original button-to-motor logic
        String[] buttons = {"cross", "circle", "triangle", "square"};
        char[] motors = {'A', 'B', 'C', 'D'};

        for (int i = 0; i < buttons.length; i++) {
            if (controllerInput.isButtonPressed(buttons[i])) {
                robotMove.setPower(motors[i], MOTOR_POWER);
                telemetry.addData("Button", "Button " + buttons[i] + " is pressed");
                telemetry.addData("Motor", "Motor " + motors[i] + " is moving");
            } else {
                robotMove.setPower(motors[i], 0);
            }
        }

        // Servo control logic
        //servoControl.update();

        telemetry.update();
    }
}
