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

    /*public class RobotMove {
        private final DcMotor motorA, motorB, motorC, motorD;
        private final Gamepad gamepad;
        private final Telemetry telemetry;
        private final ControllerInputHandler controllerInput;

        public RobotMove(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
            this.motorA = hardwareMap.get(DcMotor.class, "motorA");
            this.motorB = hardwareMap.get(DcMotor.class, "motorB");
            this.motorC = hardwareMap.get(DcMotor.class, "motorC");
            this.motorD = hardwareMap.get(DcMotor.class, "motorD");
            this.gamepad = gamepad;
            this.telemetry = telemetry;
            this.controllerInput = new ControllerInputHandler(gamepad);

            initializeMotors();
        }

        private void initializeMotors() {
            motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorA.setDirection(DcMotorSimple.Direction.REVERSE);

            motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorB.setDirection(DcMotorSimple.Direction.FORWARD);

            motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorC.setDirection(DcMotorSimple.Direction.REVERSE);

            motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorD.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public void setDefaultOrientation() {
            // Placeholder method for IMU orientation reset, add logic if required
        }

        public void setPower(char motor, double power) {
            switch (motor) {
                case 'A':
                    motorA.setPower(power);
                    telemetry.addData("Motor A Power", power);
                    break;
                case 'B':
                    motorB.setPower(power);
                    telemetry.addData("Motor B Power", power);
                    break;
                case 'C':
                    motorC.setPower(power);
                    telemetry.addData("Motor C Power", power);
                    break;
                case 'D':
                    motorD.setPower(power);
                    telemetry.addData("Motor D Power", power);
                    break;
                default:
                    telemetry.addData("Error", "Invalid motor identifier: " + motor);
                    break;
            }
            telemetry.update();
        }
    }

    public class ServoControl {
        private final Servo servo;
        private final Gamepad gamepad;
        private final Telemetry telemetry;

        public ServoControl(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
            this.servo = hardwareMap.get(Servo.class, "servo");
            this.gamepad = gamepad;
            this.telemetry = telemetry;
        }

        public void update() {
            if (gamepad.right_bumper) {
                servo.setPosition(1.0);
                telemetry.addData("Servo", "Moved to position 1.0");
            } else {
                servo.setPosition(0.0);
                telemetry.addData("Servo", "Moved to position 0.0");
            }
        }
    }*/
}
