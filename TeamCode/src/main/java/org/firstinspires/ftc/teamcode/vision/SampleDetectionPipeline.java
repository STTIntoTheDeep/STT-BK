package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.hardware.cameraAlpha;
import static org.firstinspires.ftc.teamcode.hardware.cameraXPos;
import static org.firstinspires.ftc.teamcode.hardware.cameraYPos;
import static org.firstinspires.ftc.teamcode.hardware.cameraZPos;
import static org.firstinspires.ftc.teamcode.hardware.xDegreePerPixel;
import static org.firstinspires.ftc.teamcode.hardware.xPixels;
import static org.firstinspires.ftc.teamcode.hardware.yDegreePerPixel;
import static org.firstinspires.ftc.teamcode.hardware.yPixels;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/**
 * TODO: documentation
 * @author Dean van Beek - 3977 STT
 * @author WutIsHummus / Alpeeen - 25679
 */
@Config
public class SampleDetectionPipeline extends OpenCvPipeline {
    boolean debug;
    public SampleDetectionPipeline(boolean localDebug){debug = localDebug;}
    public boolean saveRAM = true;
    public volatile double[] bestSampleInformation;
    public int count;

    /*
     * Working image buffers
     */
    Mat YCbCrMat = new Mat(),
        CrMat = new Mat(),
        CbMat = new Mat();

    Mat blueThresholdMat = new Mat(),
        redThresholdMat = new Mat(),
        yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat(),
        morphedRedThreshold = new Mat(),
        morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    RotatedRect rotatedRectFitToContour;

    ArrayList<MatOfPoint> blueContoursList = new ArrayList<>(), yellowContoursList = new ArrayList<>(), redContoursList = new ArrayList<>();

    /*
     * Threshold values
     */
    public static int
            AREA_LOWER_LIMIT = 20000,
            AREA_UPPER_LIMIT = 100000,
            YELLOW_MASK_THRESHOLD = 100,
            BLUE_MASK_THRESHOLD = 150,
            RED_MASK_THRESHOLD = 170;

    /*
     * Elements for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6.0, 6.0)),
        dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6.0, 6.0));

    /*
     * Colors for drawing on the Driver Station.
     */
    public static final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255),
            YELLOW = new Scalar(255, 255, 0),
            WHITE = new Scalar(255,255,255);

    /**
     * TODO: documentation
     */
    public class Sample {
        double grabAngle;
        Size size;
        Point cameraPosition;
        double cameraYAngle,cameraXAngle;
        double
            actualX,actualY,
            actualAngle;
        Scalar color;
        double score;

        /**
         * Is cameraY, which is robotX, and is relative to the intake.
         * FIXME
         */
        void inferY() {
            cameraYAngle = (cameraPosition.y - 0.5 * yPixels) * yDegreePerPixel;
            actualY = cameraZPos * Math.tan(Math.toRadians(cameraAlpha - cameraYAngle)) + cameraXPos;
            //TODO: correct for intake offset
        }

        /**
         * Is cameraX, which is robotY, and is relative to the intake.
         * FIXME
         */
        void inferX() {
            cameraXAngle = (cameraPosition.x - 0.5 * xPixels) * xDegreePerPixel;
            actualX = (cameraZPos / Math.cos(Math.toRadians(cameraAlpha - cameraYAngle)))*Math.tan(Math.toRadians(cameraXAngle)) * scaleX(cameraYAngle) - cameraYPos;
        }

        /**
         * Angle the robot would have to turn to align the slides with the sample.
         * FIXME
         */
        void inferAngle() {
            actualAngle = Math.tan(actualX/actualY);
        }

        /**
         * TODO: documentation
         */
        void assignSamplePoints() {
            score -= (int) actualY;
            score -= (int) Math.pow(actualX, 2);
            //FIXME
        }
        /**
         * TODO: documentation
         * TODO: delete because rectilinear
         * @param y
         * @return
         */
        double scaleX(double y) {
            return 1.75;
//            return Math.cos(Math.toRadians(9*Math.sqrt(3025.0/377.0))) * y + 1; //TODO regression
        }
    }

    ArrayList<Sample> internalSampleList = new ArrayList<>();

    /*
     * Viewport stages
     */
    enum Stage {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS
    }

    Stage[] stages = Stage.values();
    int stageNum = 0;
    public Sample bestSample;
    public volatile Scalar desiredColor = YELLOW, processingColor = YELLOW;

    @Override
    public void onViewportTapped() {
        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    /**
     * TODO: documentation
     * @param input
     * @return
     */
    @Override
    public Mat processFrame(Mat input) {
        if (saveRAM) return input;
        internalSampleList.clear();

        /*
         * Run the image processing
         */
        colorChange();
        //TODO: only calculate for desired color
        findContours(input);
        count = internalSampleList.size();
        bestSampleInformation = getBestSampleInformation(internalSampleList);
        if (debug) if (bestSample != null) if (bestSample.cameraPosition != null) Imgproc.circle(input, bestSample.cameraPosition, 15, WHITE);


        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum]) {
            case YCrCb:
                return YCbCrMat;

            case MASKS:
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);
                return masks;

            case MASKS_NR:
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);
                return masksNR;

            case CONTOURS:
                return contoursOnPlainImageMat;

            default:
                return input;
        }
    }

    private void colorChange() {
        if (desiredColor != processingColor) processingColor = desiredColor;
    }

    /**
     * TODO: documentation
     * @param input
     */
    void findContours(Mat input) {
        // Convert the input image to YCrCb color space
        //TODO: HSV for dealing with different intensities.
        Imgproc.cvtColor(input, YCbCrMat, Imgproc.COLOR_RGB2YCrCb);
//
//        if (processingColor == YELLOW) {
//            Core.extractChannel(YCbCrMat, CbMat, 2); // Cb channel index is 2
//            Imgproc.threshold(CbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
//            morphMask(yellowThresholdMat, morphedYellowThreshold);
//            yellowContoursList.clear();
//            Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//            contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());
//            for(MatOfPoint contour : yellowContoursList) {
//                analyzeContour(contour, input, YELLOW);
//            }
//        } else if (processingColor == BLUE) {
//            Core.extractChannel(YCbCrMat, CbMat, 2); // Cb channel index is 2
//            Imgproc.threshold(CbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
//            morphMask(blueThresholdMat, morphedBlueThreshold);
//            blueContoursList.clear();
//            Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//            contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());
//            for(MatOfPoint contour : blueContoursList) {
//                analyzeContour(contour, input, BLUE);
//            }
//        } else {
//            Core.extractChannel(YCbCrMat, CbMat, 2); // Cb channel index is 2
//            Imgproc.threshold(CrMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
//            morphMask(redThresholdMat, morphedRedThreshold);
//            redContoursList.clear();
//            Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//            contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());
//            for(MatOfPoint contour : redContoursList) {
//                analyzeContour(contour, input, RED);
//            }
//        }
        // Extract the Cb and Cr channels
        Core.extractChannel(YCbCrMat, CbMat, 2); // Cb channel index is 2
        Core.extractChannel(YCbCrMat, CrMat, 1); // Cr channel index is 1

        // Threshold the channels to form masks
        Imgproc.threshold(CbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(CrMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(CbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the masks
        blueContoursList.clear();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        redContoursList.clear();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        yellowContoursList.clear();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Create a plain image for drawing contours
        contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());

        // Analyze and draw contours
        for(MatOfPoint contour : blueContoursList) {
            analyzeContour(contour, input, BLUE);
        }

        for(MatOfPoint contour : redContoursList) {
            analyzeContour(contour, input, RED);
        }

        for(MatOfPoint contour : yellowContoursList) {
            analyzeContour(contour, input, YELLOW);
        }
    }

    /**
     * TODO: documentation
     * @param input
     * @param output
     */
    void morphMask(Mat input, Mat output) {
        /*
         * Apply erosion and dilation for noise reduction
         */
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    /**
     * TODO: documentation
     * @param contour
     * @param input
     * @param color
     */
    void analyzeContour(MatOfPoint contour, Mat input, Scalar color) {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Fit a rotated rectangle to the contour and draw it
        rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        if(rotatedRectFitToContour.size.area() > AREA_LOWER_LIMIT && rotatedRectFitToContour.size.area() < AREA_UPPER_LIMIT) {

            // Adjust the angle based on rectangle dimensions
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90;
            }

            // Compute the angle and store it
            double angle = -(rotRectAngle - 180);

            // Store the detected stone information
            Sample sample = new Sample();
            sample.size = rotatedRectFitToContour.size;
            sample.cameraPosition = rotatedRectFitToContour.center;
            sample.grabAngle = angle;
            sample.color = color;
            internalSampleList.add(sample);

            if(debug) {
                sample.inferY();
                sample.inferX();
                sample.assignSamplePoints();
                drawRotatedRect(rotatedRectFitToContour, input, color);
                drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color);
                drawTagText(
                    rotatedRectFitToContour,
                  (int) Math.round(angle) + " deg " +
//                  (int) Math.round(rotatedRectFitToContour.size.area()) + " area " +
                    Math.round(10 * (sample.actualX - cameraYPos)) + " x " +
                    Math.round(10 * (sample.actualY + cameraXPos)) + " y",
//                    (int) Math.round(sample.cameraXAngle) + " x " +
//                    (long) sample.cameraYAngle + " y ",
//                    sample.score + " score ",
                    input, BLUE);
            }
        }
    }

    /**
     * TODO: documentation
     * @param sampleList
     * @return
     */
    public void getBestSample(ArrayList<Sample> sampleList) {
        if (!sampleList.isEmpty()) {
            bestSample = null;
            for (Sample sample : sampleList) {
                if (sample.color == desiredColor) {
                    sample.inferY();
                    sample.inferX();
                    sample.inferAngle();
                    sample.assignSamplePoints();
                    if (bestSample == null) bestSample = sample;
                    else if (sample.score > bestSample.score) bestSample = sample;
                }

    //        sample.yFromBorder;
    //        sample.xFromBorder;
                //TODO: if too close to border remove from sampleList
            }
        } else bestSample = null;
    }

    /**
     * TODO: documentation
     * @param sampleList
     * @return
     */
    public double[] getBestSampleInformation(ArrayList<Sample> sampleList) {
        getBestSample(sampleList);
        if (bestSample == null) return null;
        return new double[] {bestSample.actualX, bestSample.actualY, bestSample.grabAngle};
    }

    /**
     * TODO: documentation
     * @param rect
     * @param text
     * @param mat
     * @param color
     */
    static void drawTagText(RotatedRect rect, String text, Mat mat, Scalar color) {

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                color, // Font color
                1); // Font thickness
        Imgproc.line(mat, new Point(224,0), new Point(224,800), color, 2);
    }

    /**
     * TODO: documentation
     * @param rect
     * @param drawOn
     * @param color
     */
    static void drawRotatedRect(RotatedRect rect, Mat drawOn, Scalar color) {
        /*
         * Draws a rotated rectangle by drawing each of the 4 lines individually
         */
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], color, 2);
            Imgproc.circle(drawOn,rect.center,15, color);
        }
    }
}