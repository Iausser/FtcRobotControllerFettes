package org.firstinspires.ftc.teamcode;

public class RobotProcesses {
    private RobotMove robotMove;
    private RobotExtras robotExtras;

    public RobotProcesses(RobotMove robotMove, RobotExtras robotExtras) {
        this.robotMove = robotMove;
        this.robotExtras = robotExtras;
    }

    public void moveRobotTime(double x, double y, double seconds) {

        long totalTime = (long) (1000 * seconds);
        long startTime = System.currentTimeMillis();
        boolean finished = false;

        while (!finished) {
            robotMove.robotCentricMovement(x, y, 0, 0);
            finished = (System.currentTimeMillis() - startTime >= totalTime);

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                // Handle the interruption (e.g., log it or throw a new exception)
                robotMove.robotCentricMovement(0, 0, 0, 0);
            }
        }

        robotMove.robotCentricMovement(0, 0, 0, 0);
    }

    public void turnToOrientation(double targetAngle, double duration) {
        long totalTime = (long) (1000 * duration);
        long startTime = System.currentTimeMillis();
        boolean finished = false;

        robotMove.autoCorrectOrientation.firstAngle = (float)robotMove.angleToRange(targetAngle);

        while (!finished) {
            // auto correct orientation to 90 degrees left facing board
            robotMove.robotCentricMovement(0, 0, 0, 0);
            finished = (System.currentTimeMillis() - startTime >= totalTime);

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                // Handle the interruption (e.g., log it or throw a new exception)
            }
        }
    }

    /*public void setArmDown() {
        robotExtras.motorArmLeft.setTargetPosition(-162);
        robotExttas.motorArmRight.setTargetPosition(-90);

    }

    public void setArmBoard() {
        robotArm.motorArmLeft.setTargetPosition(-85);
        robotArm.motorArmRight.setTargetPosition(-13);
    }

    public void pickUpPixel() {
        // make sure arm is in down position first
        setArmDown();
    }*/

}
