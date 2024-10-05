package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotExtras {
    private HardwareMap hardwareMap;
    private Gamepad gamepad;
    private Telemetry telemetry;
    public Servo servoPixel, servoDrone;
    public DcMotor motorBrush;
    public static final double SERVO_PIXEL_CLOSED = 0.0;
    public static final double SERVO_PIXEL_OPEN = 0.055;
    public static final double SERVO_DRONE_CLOSED = 0.0;
    public static final double SERVO_DRONE_OPEN = 2.2;

    public Button brushButton, droneButton, reverseBrushButton;
    private static final double BRUSH_POWER = 0.5;
    private ControllerInputHandler controllerInput;
    public RobotExtras(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        servoPixel = hardwareMap.get(Servo.class, "servoPixel");
        motorBrush = hardwareMap.get(DcMotor.class, "motorBrush");

        motorBrush.setDirection(DcMotorSimple.Direction.REVERSE);
        servoPixel.setPosition(SERVO_PIXEL_OPEN);

        servoDrone = hardwareMap.get(Servo.class, "servoDrone");
        servoDrone.setPosition(SERVO_DRONE_CLOSED);

        controllerInput = new ControllerInputHandler(gamepad);
        brushButton = new Button("leftstickbutton", false);
        droneButton = new Button("square", false);
        reverseBrushButton = new Button("rightstickbutton", false);
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void releaseDrone() {
        servoDrone.setPosition(SERVO_DRONE_OPEN);
    }

    public void doHardwareMovement() {
        controllerInput.updateButton(brushButton);
        motorBrush.setPower(brushButton.onMode ? BRUSH_POWER : 0);

        if (controllerInput.updateButton(reverseBrushButton) && reverseBrushButton.onMode == false) {
            motorBrush.setPower(0);
        }
        if (reverseBrushButton.onMode) {
            motorBrush.setPower(-BRUSH_POWER);
        }

        if (controllerInput.updateButton(droneButton)) {
            telemetry.addData("Launching drone", "");
            releaseDrone();
        }
    }
}
