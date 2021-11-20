package org.firstinspires.ftc.teamcode.freightdetection;

import org.opencv.calib3d.Calib3d;
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



    Mat cbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat cannyOutput = new Mat();
    Mat contoursOnPlainMat = new Mat();



    ArrayList<MatOfPoint> contoursList = new ArrayList<>();

    static final int CB_THRESHOLD = 80;

    static final int minimumRadius = 70;
    static final int minimumSideLength = 70;

    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    float[][] circleRadii;
    Point[] circleCenters;
    Rect[] boundingRects;

    enum Stage {
        FINAL("Contours/Bounding Boxes"),
        CB("Extracted Cb"),
        THRESHOLD("Binary Threshold"),
        THRESHOLD_MORPH("Noise Reduction"),
        CANNY("Canny Edge Detection"),
        CONTOURS_ON_INPUT("Contours Drawn");

        String text;

        public void putText(Mat input){
            Imgproc.putText(
                    input,
                    this.text,
                    new Point(70, input.cols()+20),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1,
                    new Scalar(235, 9, 54),
                    5
            );
        }

        Stage(String text){
            this.text = text;
        }
    }

    Stage[] stages = Stage.values();

    int stageNum = 0;

    @Override
    public void onViewportTapped() {
        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length){
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    private void drawCircle(Mat input, int i){
        if(isCircleLargeEnough(circleRadii[i][0])) {
            Imgproc.circle(input, circleCenters[i], (int) circleRadii[i][0], TEAL, 3);
        }
    }

    private void drawRectangle(Mat input, int i){
        if(isRectangleLargeEnough(boundingRects[i])) {
            Imgproc.rectangle(input, boundingRects[i].tl(), boundingRects[i].br(), PURPLE, 3);
        }
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

        contoursList = findContours(input);

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contoursList.size()];
        circleCenters = new Point[contoursList.size()];
        circleRadii = new float[contoursList.size()][1];
        boundingRects = new Rect[contoursList.size()];

        for (int i = 0; i < contoursList.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursList.get(i).toArray()), contoursPoly[i], 5, true);
            boundingRects[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            circleCenters[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], circleCenters[i], circleRadii[i]);
        }

        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        for (int i = 0; i < contoursList.size(); i++) {
            Imgproc.drawContours(input, contoursPolyList, i, new Scalar(255, 186, 3), 2);
            drawCircle(input, i);
            drawRectangle(input, i);
        }

        switch (stages[stageNum]){
            case CB:
                return cbMat;
            case FINAL:
                return input;
            case THRESHOLD:
                return thresholdMat;
            case THRESHOLD_MORPH:
                return morphedThreshold;
            case CANNY:
                return cannyOutput;
            case CONTOURS_ON_INPUT:
                return contoursOnPlainMat;
        }

        Arrays.stream(contoursPoly).forEach(Mat::release);
        contoursPolyList.forEach(Mat::release);
        contoursList.forEach(Mat::release);
        if(stages[stageNum] != Stage.FINAL) input.release();

        return input;
    }

    ArrayList<MatOfPoint> findContours(Mat input){
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();

        //Extract the Cb channel of the frame
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat,2);

        //Threshold the channel and then use some noise reduction
        Imgproc.Canny(cbMat, cannyOutput, CB_THRESHOLD, CB_THRESHOLD * 2.0);

        //Not used but meh
        Imgproc.threshold(cbMat, thresholdMat, CB_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.erode(cannyOutput, morphedThreshold, erodeElement);
        Imgproc.erode(morphedThreshold, morphedThreshold, erodeElement);

        Imgproc.dilate(morphedThreshold, morphedThreshold, dilateElement);
        Imgproc.dilate(morphedThreshold, morphedThreshold, dilateElement);

        //Find the contours
        Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        input.copyTo(contoursOnPlainMat);
        Imgproc.drawContours(contoursOnPlainMat, contoursList, -1, BLUE, 2, 8);


        return contoursList;


    }
}
