package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends OpMode {
    private ControllerInputHandler controllerInput;
    private RobotProcesses robotProcesses;
    private RobotMove robotMove;
    private RobotExtras robotExtras;

    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        robotMove = new RobotMove(hardwareMap, gamepad1, telemetry);
        robotExtras = new RobotExtras(hardwareMap, gamepad1, telemetry);
        robotProcesses = new RobotProcesses(robotMove, robotExtras);
    }

    @Override
    public void loop() {
        doAutonomousMode();
        return;
    }

    private void doAutonomousMode() {
        robotProcesses.moveRobotTime(0, 1, 1);
    }
}