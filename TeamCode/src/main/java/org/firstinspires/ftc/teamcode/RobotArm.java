package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArm {
    public DcMotor motorArmLeft, motorArmRight;
    public Servo servoArm, servoHand;
    private Gamepad gamepad;
    private ControllerInputHandler controllerInput;
    private static final double MOTOR_ARM_POWER = 0.26;
    private static final double SERVO_ARM_ANGLE_INCREMENT = 0.025;
    public static final double SERVO_HAND_CLOSED = 0.0;
    public static final double SERVO_HAND_OPEN = 0.056;
    public static final double SERVO_ARM_DOWN = 1.00;//0.55 ;
    public static final double SERVO_ARM_UP = 1.125;
    public static final double SERVO_ARM_BOARD = 0.475;
    public static final int MOTOR_ARM_DOWN = -8;
    public static final int MOTOR_ARM_UP = 96;
    public static final int MOTOR_ARM_BOARD = 118;
    private Telemetry telemetry;
    public Button handToggleButton, motorArmUpButton, motorArmDownButton, servoArmUpButton, servoArmDownButton, armDownButton, armUpButton, armBoardButton;
    public double handAngle, servoArmAngle;

    public RobotArm(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorArmLeft = hardwareMap.get(DcMotor.class, "motorArmLeft");
        motorArmRight = hardwareMap.get(DcMotor.class, "motorArmRight");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoHand = hardwareMap.get(Servo.class, "servoHand");

        this.gamepad = gamepad;
        this.telemetry = telemetry;

        controllerInput = new ControllerInputHandler(gamepad);
        motorArmUpButton = new Button("leftbumper", false);
        motorArmDownButton = new Button("rightbumper", false);
        servoArmUpButton = new Button("lefttrigger", false);
        servoArmDownButton = new Button("righttrigger", false);
        handToggleButton = new Button("circle", false);

        motorArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        initialiseMotors();
        handAngle = 0;
        servoArmAngle = SERVO_ARM_DOWN;
    }

    private void initialiseMotors() {
        motorArmLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArmLeft.setPower(MOTOR_ARM_POWER);
        motorArmRight.setPower(MOTOR_ARM_POWER);

        servoArm.setDirection(Servo.Direction.FORWARD);
        servoHand.setDirection(Servo.Direction.FORWARD);
        servoHand.setPosition(SERVO_HAND_CLOSED);
        servoArm.setPosition(SERVO_ARM_DOWN);
    }

    public void openHand() {servoHand.setPosition(SERVO_HAND_OPEN);}
    public void closeHand() {servoHand.setPosition(SERVO_HAND_CLOSED);}


    public void moveArmBoard() {
        servoArm.setPosition(SERVO_ARM_BOARD);
        motorArmLeft.setTargetPosition(MOTOR_ARM_BOARD);
        motorArmRight.setTargetPosition(MOTOR_ARM_BOARD);
    }

    public void doArmMovement() {
        // update arm buttons
        controllerInput.updateButton(motorArmUpButton);
        controllerInput.updateButton(motorArmDownButton);

        // hand buttons
        if (controllerInput.updateButton(handToggleButton)) {
            servoHand.setPosition(handToggleButton.onMode ? SERVO_HAND_OPEN : SERVO_HAND_CLOSED);
        }

        // servo arm buttons
        if (controllerInput.updateButton(servoArmUpButton)) {
            servoArmAngle += SERVO_ARM_ANGLE_INCREMENT;
            servoArm.setPosition(servoArmAngle);
            telemetry.addData("arm going up", "");
        }
        if (controllerInput.updateButton(servoArmDownButton)) {
            servoArmAngle -= SERVO_ARM_ANGLE_INCREMENT;
            if (servoArmAngle < 0) servoArmAngle = 0;   // set limit at position 0
            servoArm.setPosition(servoArmAngle);
            telemetry.addData("arm going down", "");
        }

        motorArmLeft.setPower(motorArmUpButton.isPressed ? MOTOR_ARM_POWER : (motorArmDownButton.isPressed ? -MOTOR_ARM_POWER : 0));
        motorArmRight.setPower(motorArmUpButton.isPressed ? MOTOR_ARM_POWER : (motorArmDownButton.isPressed ? -MOTOR_ARM_POWER : 0));
    }
}


