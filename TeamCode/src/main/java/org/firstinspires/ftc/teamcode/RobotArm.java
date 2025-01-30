package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArm {

    // Hardware components
    private DcMotor motorArm;
    private DcMotor motorJoint;
    private Servo servoHand;

    // Gamepad and controller handler
    private final Gamepad gamepad;
    private final ControllerInputHandler controllerInput;

    // Constants for motor power and servo positions
    private static final double MOTOR_ARM_SCALING = 0.7;
    private static final double SERVO_HAND_CLOSED = 0.0;
    private static final double SERVO_HAND_OPEN = 0.5;
    private static final double JOINT_POWER = 0.5;

    // Telemetry for runtime feedback
    private final Telemetry telemetry;

    // Buttons for toggling and motor control
    private final Button handToggleButton;
    private final Button servoArmToggleButton; // Now mapped to the square button

    // States for toggling
    private boolean isHandOpen = true; // Default state for hand is open
    private boolean isServoArmUp = false;

    // Constructor
    public RobotArm(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        // Initialize hardware components
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorJoint = hardwareMap.get(DcMotor.class, "motorJoint");
        servoHand = hardwareMap.get(Servo.class, "servoHand");

        this.gamepad = gamepad;
        this.telemetry = telemetry;

        // Initialize controller inputs and buttons
        controllerInput = new ControllerInputHandler(gamepad);
        servoArmToggleButton = new Button("square", false);
        handToggleButton = new Button("rightbumper", false);

        // Configure motors and servos
        initialiseMotors();
    }

    private void initialiseMotors() {
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorJoint.setDirection(DcMotorSimple.Direction.FORWARD);
        motorJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoHand.setPosition(SERVO_HAND_CLOSED); // Default hand state
    }

    private void toggleServoArm() {
        if (controllerInput.updateButton(servoArmToggleButton)) {
            isServoArmUp = !isServoArmUp;
            telemetry.addData("Arm Position", isServoArmUp ? "Up" : "Down");
        }
    }

    private void toggleServoHand() {
        if (controllerInput.updateButton(handToggleButton)) {
            isHandOpen = !isHandOpen;
            servoHand.setPosition(isHandOpen ? SERVO_HAND_OPEN : SERVO_HAND_CLOSED);
            telemetry.addData("Hand State", isHandOpen ? "Open" : "Closed");
        }
    }

    private void controlMotorJoint() {
        // Check which bumper is pressed and set the motor power accordingly
        if (gamepad.right_bumper) {
            motorJoint.setPower(JOINT_POWER); // Forward power
        } else if (gamepad.left_bumper) {
            motorJoint.setPower(-JOINT_POWER); // Reverse power
        } else {
            motorJoint.setPower(0); // Stop the motor when neither bumper is pressed
        }
    }

    public void doArmMovement() {
        // Update toggles
        toggleServoHand();
        toggleServoArm();

        // Control motorJoint
        controlMotorJoint();

        // Calculate motor power for motorArm
        double power = (gamepad.right_trigger - gamepad.left_trigger) * MOTOR_ARM_SCALING;
        motorArm.setPower(power);

        // Telemetry feedback
        telemetry.addData("Motor Power", power);
        telemetry.addData("Left Trigger", gamepad.left_trigger);
        telemetry.addData("Right Trigger", gamepad.right_trigger);
        telemetry.addData("Hand State", isHandOpen ? "Open" : "Closed");
        telemetry.addData("Arm Position", isServoArmUp ? "Up" : "Down");
        telemetry.update();
    }
}
