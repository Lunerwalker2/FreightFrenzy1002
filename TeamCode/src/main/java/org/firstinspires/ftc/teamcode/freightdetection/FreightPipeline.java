package org.firstinspires.ftc.teamcode.freightdetection;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
public class FreightPipeline extends OpenCvPipeline {


    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));




    Mat blurredMat = new Mat();
    Mat extractedMat = new Mat();
//    Mat cannyOutput = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainMat = new Mat();

    boolean detectingSilver = false;



    ArrayList<MatOfPoint> contoursList = new ArrayList<>();

    static final int CB_THRESHOLD = 80;
    static final int GREY_THRESHOLD = 180;

    static final int minimumRadius = 50;
    static final int maximumRadius = 150;
    static final int minimumSideLength = 20;
    static final int maximumSideLength = 200;

    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    float[][] circleRadii;
    Point[] circleCenters;
    Rect[] boundingRects;

    enum Stage {
        FINAL("Final"),
        BLURRED_INPUT("Noise Reduction"),
        EXTRACTED_CHANNEL("Extracted Channel"),
        THRESHOLD("Binary Threshold"),
        THRESHOLD_MORPH("Morphed Threshold"),
        CONTOURS_ON_INPUT("Contours Drawn");

        private final String text;


        public void putText(Mat input){
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

        Stage(String text){
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

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    private void drawCircle(Mat input, int i){
            Imgproc.circle(input, circleCenters[i], (int) circleRadii[i][0], TEAL, 3);
            drawTagText(input, "Sphere", circleCenters[i]);
    }

    private void drawRectangle(Mat input, int i){
            Imgproc.rectangle(input, boundingRects[i].tl(), boundingRects[i].br(), PURPLE, 3);
            drawTagText(input, "Cube", centerOfRect(boundingRects[i]));
    }

    static Point centerOfRect(Rect rect){
        return new Point(
                (int) (rect.x + (rect.width / 2)),
                (int) (rect.y + (rect.height / 2))
        );
    }

    private boolean isCircleLargeEnough(double radius){
        double diameter = 2*radius;
        return (diameter > minimumRadius);
    }

    private boolean isRectangleLargeEnough(Rect rect){
        return (rect.height > minimumSideLength) && (rect.width > minimumSideLength);
    }


    @Override
    public Mat processFrame(Mat input){

        contoursList.clear();

        //do an initial blur
        blur(input, blurredMat);

        //extract either the alpha channel for balls or cb channel for cubes
        if(detectingSilver) extractAlpha(blurredMat, extractedMat);
        else extractCb(blurredMat, extractedMat);


        //Use a threshold for now on both cases
        if(detectingSilver){
            Imgproc.threshold(extractedMat, thresholdMat, GREY_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        } else {
            Imgproc.threshold(extractedMat, thresholdMat, CB_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        }

        //further reduce noise
        morph(thresholdMat, morphedThreshold);

        if(detectingSilver){
            //find only the outer contours
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        } else {
            //find yellow blobs, may have to morph mask more for this to work better
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        //draw the contours we find
        input.copyTo(contoursOnPlainMat);
        Imgproc.drawContours(contoursOnPlainMat, contoursList, -1, BLUE, 2, 8);

        //make a list to hold polygon objects
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursList.size()];

        if(detectingSilver) {
            circleCenters = new Point[contoursList.size()];
            circleRadii = new float[contoursList.size()][1];
        } else {
            boundingRects = new Rect[contoursList.size()];
        }

        //Find shapes in the contours and make either rects or circles
        for (int i = 0; i < contoursList.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursList.get(i).toArray()), contoursPoly[i], 5, true);
            if(detectingSilver){
                circleCenters[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], circleCenters[i], circleRadii[i]);
            } else {
                boundingRects[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }
        }

        //Change the shapes into something we can draw
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        for (int i = 0; i < contoursList.size(); i++) {
            //Draw the shapes
            Imgproc.drawContours(input, contoursPolyList, i, new Scalar(255, 186, 3), 2);
            if(detectingSilver) {
                //Check if the circle is big enough
                if(isCircleLargeEnough(circleRadii[i][0])){
                    drawCircle(input, i);
                }
            } else {
                //Check if the rectangle is big enough
                if(isRectangleLargeEnough(boundingRects[i])) {
                    drawRectangle(input, i);
                }
            }
        }

        //Make sure we release anything that's a mat, and the input frame if its not being returned
        Arrays.stream(contoursPoly).forEach(Mat::release);
        contoursPolyList.forEach(Mat::release);
        contoursList.forEach(Mat::release);
        if(stages[stageNum] != Stage.FINAL) input.release();

        //Write on each mat and then return it
        switch (stageToRenderToViewport){
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

    void blur(Mat src, Mat dst){
    //        Imgproc.GaussianBlur(
    //                src,
    //                dst,
    //                new Size(5,5),
    //                0,
    //                0
    //        );
        Imgproc.bilateralFilter(
                src,
                dst,
                4,
                25,
                15
        );
    }

    void morph(Mat src, Mat dst){
        Imgproc.erode(src, dst, erodeElement);
        Imgproc.erode(dst, dst, erodeElement);

        Imgproc.dilate(dst, dst, dilateElement);
        Imgproc.dilate(dst, dst, dilateElement);
    }

    void extractCb(Mat src, Mat dst){
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(dst, dst, 2);
    }

    void extractAlpha(Mat src, Mat dst){
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2GRAY);
        Core.extractChannel(dst, dst, 0);
    }

    void drawTagText(Mat input, String text, Point point){
        Imgproc.putText(
                input, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        point.x-5,  // x anchor point
                        point.y-10), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1.2, // Font size
                TEAL, // Font color
                2); // Font thickness
    }
}
