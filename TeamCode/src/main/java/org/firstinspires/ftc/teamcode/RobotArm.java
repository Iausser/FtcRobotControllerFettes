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
    private static final double MOTOR_JOINT_SCALING = 0.4;

    // Telemetry for runtime feedback
    private final Telemetry telemetry;

    // States for toggling
    private boolean isHandOpen = true; // Default state for hand is open
    private boolean squareButtonLastState = false; // Prevent multiple toggles per press

    // motor variables
    private int lastJointPosition = 0;

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

        // Configure motors and servos
        initialiseMotors();
    }

    private void initialiseMotors() {
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorJoint.setDirection(DcMotorSimple.Direction.FORWARD);
        motorJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoHand.setPosition(SERVO_HAND_CLOSED); // Default hand state
    }

    private void toggleServoHand() {
        boolean squareButton = controllerInput.isButtonPressed("Square");

        if (squareButton && !squareButtonLastState) { // Detect fresh press
            isHandOpen = !isHandOpen;
            servoHand.setPosition(isHandOpen ? SERVO_HAND_OPEN : SERVO_HAND_CLOSED);
            telemetry.addData("Hand State", isHandOpen ? "Open" : "Closed");
        }

        squareButtonLastState = squareButton; // Update last state
    }

    private void controlMotorArm() {
        double power = ((gamepad.right_bumper ? 1 : 0) - (gamepad.left_bumper ? 1 : 0)) * MOTOR_ARM_SCALING;
        motorArm.setPower(power);
    }

    private void controlMotorJoint() {
        // Left trigger = arm down, right trigger = arm up
        double power = (gamepad.right_trigger - gamepad.left_trigger) * MOTOR_JOINT_SCALING;
        motorJoint.setPower(power);

        if (power == 0) {
            // Hold last known position only when there is no user input
            lastJointPosition = motorJoint.getCurrentPosition();
            motorJoint.setTargetPosition(lastJointPosition);
            motorJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorJoint.setPower(1.0);
        } else {
            // When moving, use normal mode
            motorJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void doArmMovement() {
        // Update toggles
        toggleServoHand();

        // Control motor movements
        controlMotorJoint();
        controlMotorArm();

        // Telemetry feedback
        telemetry.addData("Arm Power", motorArm.getPower());
        telemetry.addData("Joint Power", motorJoint.getPower());
        telemetry.addData("Servo Position", servoHand.getPosition());
        telemetry.update();
    }
}
