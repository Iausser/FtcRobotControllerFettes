package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMove {
    private final DcMotor motorA, motorB, motorC, motorD;
    private Servo servoDrone;
    private static final double MAX_AVAILABLE_POWER = 0.98;   // 2% reduction in max power
    private static final double MAX_MOTOR_POWER = 0.9 * MAX_AVAILABLE_POWER;   // don't use all available power (too sensitive)
    private static final double TURN_SCALAR = 0.6;    // turning scalar (can be adjusted)
    private BHI260IMU bhi260; // Using the BHI260IMU sensor on the control hub
    private Orientation defaultOrientation;
    private ControllerInputHandler controllerInput;
    private Gamepad gamepad;
    public Telemetry telemetry;
    public Button robotCentricMovement, fieldCentricMovement, orientationButton;
    public Orientation autoCorrectOrientation;
    private boolean isTurning;
    private static final double AUTO_CORRECT_SENSITIVITY = 3.0;
    private static final double TWO_PI = 2 * Math.PI;

    public RobotMove(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        motorD = hardwareMap.get(DcMotor.class, "motorD");
        bhi260 = hardwareMap.get(BHI260IMU.class, "imu");
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        controllerInput = new ControllerInputHandler(gamepad);
        robotCentricMovement = new Button("square", true);
        fieldCentricMovement = new Button("square", false);
        orientationButton = new Button("cross", false);

        initialiseMotors();
        initialiseIMU();
        //servoDrone = hardwareMap.get(Servo.class, "servoDrone");

        defaultOrientation = getIMUOrientation(); // Initialize defaultOrientation
        autoCorrectOrientation = getIMUOrientation();
        isTurning = true;   // initialise to true so that auto-correct turning can start immediately
    }

    private void initialiseMotors() {
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setDirection(DcMotorSimple.Direction.REVERSE);

        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setDirection(DcMotorSimple.Direction.FORWARD);

        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setDirection(DcMotorSimple.Direction.REVERSE);

        motorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorD.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initialiseIMU() {
        bhi260.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
    }

    // gives power to any wheel motor
    public void setPower(char motor, double power) {
        switch (motor) {
            case 'A':
                motorA.setPower(power);
                break;
            case 'B':
                motorB.setPower(power);
                break;
            case 'C':
                motorC.setPower(power);
                break;
            case 'D':
                motorD.setPower(power);
                break;
        }
    }

    // converts joystick coords to an angle
    private double xy_to_angle(double x, double y) {
        if (x >= 0 && y == 0) return 0.0;
        if (x == 0 && y > 0) return Math.PI / 2;
        if (x < 0 && y == 0) return Math.PI;
        if (x == 0 && y < 0) return 3 * Math.PI / 2;
        return Math.atan2(y, x);
    }

    public double angleToRange(double angle) {
        angle %= (Math.PI * 2);
        if (angle >= Math.PI) return angle -= Math.PI * 2;
        return angle;
    }

    // sets the motors to move orthogonally at some angle and power value while turning with speed turn_value
    public void robotCentricMovement(double x, double y, double offset_angle, double turn_value) {
        double theta = xy_to_angle(x, y) - offset_angle;
        double power = Math.sqrt(x*x + y*y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // orthogonal movement (normalised sin and cos to make use of all available power)
        // multiply by MAX_MOTOR_POWER to account for any variation in motor strength (2% assumed) + sensitivity
        double speed_a = power * cos/max * MAX_MOTOR_POWER;
        double speed_b = power * sin/max * MAX_MOTOR_POWER;
        double speed_c = power * sin/max * MAX_MOTOR_POWER;
        double speed_d = power * cos/max * MAX_MOTOR_POWER;

        // add auto-correct turning
        // there is some error occurring here
        // robot keeps turning for some reason after turn_value is set to zero
        // perhaps the auto-correct orientation isn't being set when it should be
        if (isTurning && turn_value == 0) {
            autoCorrectOrientation = getIMUOrientation();
        }
        isTurning = turn_value != 0;

        if (isTurning == false) {
            // get orientation of the robot relative to its movement direction using IMU
            Orientation currentOrientation = getIMUOrientation();
            double deltaAngle = currentOrientation.firstAngle - autoCorrectOrientation.firstAngle;

            // auto adjust for being off using turning
            turn_value = angleToRange(deltaAngle) * AUTO_CORRECT_SENSITIVITY;
        }

        // add turning
        speed_a += turn_value * TURN_SCALAR;
        speed_b -= turn_value * TURN_SCALAR;
        speed_c += turn_value * TURN_SCALAR;
        speed_d -= turn_value * TURN_SCALAR;

        // account for any power overshooting
        if ((power * MAX_MOTOR_POWER + Math.abs(turn_value)) > MAX_MOTOR_POWER) {
            speed_a /= (power + Math.abs(turn_value));
            speed_b /= (power + Math.abs(turn_value));
            speed_c /= (power + Math.abs(turn_value));
            speed_d /= (power + Math.abs(turn_value));
        }

        // set motor speeds
        motorA.setPower(speed_a);
        motorB.setPower(speed_b);
        motorC.setPower(speed_c);
        motorD.setPower(speed_d);
    }

    // the same as robot centric movement except controls work relative to the field instead of the robot
    public void fieldCentricMovement(double x, double y, double turn_value) {
        // get orientation of the robot relative to the field using IMU
        Orientation currentOrientation = getIMUOrientation();
        double deltaAngle = currentOrientation.firstAngle - defaultOrientation.firstAngle;

        // do movement with new angle
        robotCentricMovement(x, y, deltaAngle, turn_value);
    }

    public void setDefaultOrientation() {
        defaultOrientation = getIMUOrientation();
    }

    // gets the current orientation of the robot
    public Orientation getIMUOrientation() {
        return bhi260.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public void doRobotMovement() {
        double leftStickX = controllerInput.getLeftStickX();
        double leftStickY = controllerInput.getLeftStickY();    // also negate the sign
        double rightStickX = controllerInput.getRightStickX();

        if (fieldCentricMovement.onMode) {
            fieldCentricMovement(leftStickX, leftStickY, rightStickX);
        } else {
            robotCentricMovement(leftStickX, leftStickY, 0, rightStickX);
        }
    }
}


