package org.firstinspires.ftc.teamcode.duck;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
Pipeline to find yellow cubes.
 */
public class DuckPipeline extends OpenCvPipeline {

    static class DuckDetection {

        public Point center;
        public float radii;
        public Pose2d relativePose;

        public DuckDetection(Point center, float radii, Pose2d relativePose){

            this.center = center;
            this.radii = radii;
            this.relativePose = relativePose;
        }
    }


    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));


    Mat blurredMat = new Mat();
    Mat extractedMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainMat = new Mat();

    private DuckDetection duckDetection;
    private Object duckDetectionLock;

    public DuckDetection getDuckDetection(){
        synchronized (duckDetectionLock){
            return duckDetection;
        }
    }


    ArrayList<MatOfPoint> contoursList = new ArrayList<>();

    public static int CB_THRESHOLD = 80;

    public static int minimumRadius = 50;
    public static int maximumRadius = 150;

    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    float[][] circleRadii;
    Point[] circleCenters;

    enum Stage {
        FINAL("Final"),
        BLURRED_INPUT("Noise Reduction"),
        EXTRACTED_CHANNEL("Extracted Channel"),
        THRESHOLD("Binary Threshold"),
        THRESHOLD_MORPH("Morphed Threshold"),
        CONTOURS_ON_INPUT("Contours Drawn");

        private final String text;


        public void putText(Mat input) {
            Imgproc.putText(
                    input,
                    this.text,
                    new Point(70, 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    new Scalar(235, 9, 54),
                    2
            );
        }

        Stage(String text) {
            this.text = text;
        }
    }

    private Stage stageToRenderToViewport = Stage.FINAL;
    Stage[] stages = Stage.values();

    int stageNum = 0;

    @Override
    public void onViewportTapped() {

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    private void drawCircle(Mat input, int i) {
        Imgproc.circle(input, circleCenters[i], (int) circleRadii[i][0], TEAL, 3);
        drawTagText(input, "Sphere", circleCenters[i]);
    }

    private boolean isCircleLargeEnough(double radius) {
        double diameter = 2 * radius;
        return (diameter > minimumRadius);
    }


    @Override
    public Mat processFrame(Mat input) {

        contoursList.clear();

        //do an initial blur
        blur(input, blurredMat);

        //extract the blue channel to search for yellow
        extractCb(blurredMat, extractedMat);


        //Use a threshold for
        Imgproc.threshold(extractedMat, thresholdMat, CB_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        //further reduce noise
        morph(thresholdMat, morphedThreshold);

        //find yellow blobs, may have to morph mask more for this to work better
        Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        //draw the contours we find
        input.copyTo(contoursOnPlainMat);
        Imgproc.drawContours(contoursOnPlainMat, contoursList, -1, BLUE, 2, 8);

        //make a list to hold polygon objects
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursList.size()];

        //Create the lists for circle centers and radii
        circleCenters = new Point[contoursList.size()];
        circleRadii = new float[contoursList.size()][1];

        //Find shapes in the contours and make circles
        for (int i = 0; i < contoursList.size(); i++) {
            //Create a list of polygons
            contoursPoly[i] = new MatOfPoint2f();
            //Make a closed shape out of the contour
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursList.get(i).toArray()), contoursPoly[i], 5, true);
            circleCenters[i] = new Point();
            //Find the center and radii of the bounding circle
            Imgproc.minEnclosingCircle(contoursPoly[i], circleCenters[i], circleRadii[i]);
        }

        //Change the shapes into something we can draw
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        for (int i = 0; i < contoursList.size(); i++) {
            //Draw the shapes
            Imgproc.drawContours(input, contoursPolyList, i, new Scalar(255, 186, 3), 2);
            //Check if the circle is big enough
            if (isCircleLargeEnough(circleRadii[i][0])) {
                drawCircle(input, i);
            }
        }

        //Make sure we release anything that's a mat, and the input frame if its not being returned
        Arrays.stream(contoursPoly).forEach(Mat::release);
        contoursPolyList.forEach(Mat::release);
        contoursList.forEach(Mat::release);
        if (stages[stageNum] != Stage.FINAL) input.release();

        //Write on each mat and then return it
        switch (stageToRenderToViewport) {
            case FINAL:
                stageToRenderToViewport.putText(input);
                return input;
            case BLURRED_INPUT:
                stageToRenderToViewport.putText(blurredMat);
                return blurredMat;
            case EXTRACTED_CHANNEL:
                stageToRenderToViewport.putText(extractedMat);
                return extractedMat;
            case THRESHOLD:
                stageToRenderToViewport.putText(thresholdMat);
                return thresholdMat;
            case THRESHOLD_MORPH:
                stageToRenderToViewport.putText(morphedThreshold);
                return morphedThreshold;
            case CONTOURS_ON_INPUT:
                stageToRenderToViewport.putText(contoursOnPlainMat);
                return contoursOnPlainMat;
            default:
                return input;
        }

    }

    void blur(Mat src, Mat dst) {
        Imgproc.bilateralFilter(
                src,
                dst,
                4,
                25,
                15
        );
        Imgproc.GaussianBlur(
                src,
                dst,
                new Size(5, 5),
                0,
                0
        );

    }

    void morph(Mat src, Mat dst) {
        Imgproc.erode(src, dst, erodeElement);
        Imgproc.erode(dst, dst, erodeElement);

        Imgproc.dilate(dst, dst, dilateElement);
        Imgproc.dilate(dst, dst, dilateElement);
    }

    void extractCb(Mat src, Mat dst) {
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(dst, dst, 2);
    }


    void drawTagText(Mat input, String text, Point point) {
        Imgproc.putText(
                input, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        point.x - 30,  // x anchor point
                        point.y - 50), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1.2, // Font size
                TEAL, // Font color
                2); // Font thickness
    }
}
