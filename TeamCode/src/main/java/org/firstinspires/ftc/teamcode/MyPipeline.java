package org.firstinspires.ftc.teamcode;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class MyPipeline extends OpenCvPipeline {
    // HSV thresholds for detecting the object (you can adjust these values)
    private static final Scalar LOWER_BOUNDS = new Scalar(30, 100, 100);
    private static final Scalar UPPER_BOUNDS = new Scalar(70, 255, 255);

    private boolean objectDetected;
    private Mat currentFrame; // To store the current frame

    public Mat processFrame(Mat input) {
        // Store the current frame
        currentFrame = input.clone();

        // Convert input frame to HSV color space
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Threshold the HSV image to get only the object color
        Mat mask = new Mat();
        Core.inRange(hsvFrame, LOWER_BOUNDS, UPPER_BOUNDS, mask);

        // Perform morphological operations to clean up the mask
        Mat morphOutput = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, morphOutput, Imgproc.MORPH_OPEN, kernel);

        // Find contours in the mask
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Check if any contours (objects) are found
        objectDetected = !contours.isEmpty();

        return input;
    }

    // Method to check if an object is detected
    public boolean objectDetected() {
        return objectDetected;
    }

    // Method to get the current frame from the camera
    public Mat getCameraFrame() {
        return currentFrame;
    }
}