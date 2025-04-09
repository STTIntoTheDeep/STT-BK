package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * TODO: documentation
 */
public class SpecimenDetectionPipeline extends OpenCvPipeline {
    Scalar lowerMask, upperMask;
    public SpecimenDetectionPipeline(boolean redAlliance) {
        if (redAlliance) {
            lowerMask = new Scalar(0, 0, 60);
            upperMask = new Scalar(80, 110, 110);
        } else {
            lowerMask = new Scalar(200, 0, 20);
            upperMask = new Scalar(240, 100, 100);
        }
    }
    double cX = 0;
    double cY = 0;
    double width = 0;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 1.5;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    @Override
    public Mat processFrame(Mat input) {
        //TODO: make work

        // Preprocess the frame to detect yellow regions
        Mat maskedInput = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(maskedInput, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);
        if (largestContour == null) return input;

        // Draw a red outline around the largest detected object
        Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
        // Calculate the width of the bounding box
        width = calculateWidth(largestContour);

        // Display the width next to the label
        String widthLabel = "Width: " + (int) width + " pixels";
        Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        //Display the Distance
        String distanceLabel = "Distance: " + String.format(Locale.ENGLISH, "%.2f", getDistance()) + " inches";
        Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        // Calculate the centroid of the largest contour
        Moments moments = Imgproc.moments(largestContour);
        cX = moments.get_m10() / moments.get_m00();
        cY = moments.get_m01() / moments.get_m00();

        // Draw a dot at the centroid
        String label = "(" + (int) cX + ", " + (int) cY + ")";
        Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerMask, upperMask, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public double getDistance() {
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }
}