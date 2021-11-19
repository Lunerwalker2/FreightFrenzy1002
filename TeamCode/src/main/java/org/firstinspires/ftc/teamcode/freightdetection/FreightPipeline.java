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

/*
Pipeline to find yellow cubes.
 */
public class FreightPipeline extends OpenCvPipeline {

    static class CubeFreight {
        double distance;
        double angle;
    }


    private Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    private Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));



    Mat cbMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat cannyOutput = new Mat();
    Mat contoursOnPlainMat = new Mat();


    ArrayList<CubeFreight> internalFreights = new ArrayList<>();
    volatile ArrayList<CubeFreight> freights = new ArrayList<>();

    static final int CB_THRESHOLD = 80;

    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);

    enum Stage {
        FINAL,
        CB,
        THRESHOLD,
        THRESHOLD_MORPH,
        CANNY,
        CONTOURS_ON_INPUT
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

    @Override
    public Mat processFrame(Mat input){

        internalFreights.clear();

        for(MatOfPoint contour : findContours(input)){
            Point[] contourArray = contour.toArray();

            if(contourArray.length >= 15){
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                Rect rect = Imgproc.boundingRect(areaPoints);

                internalFreights.add()
            }
        }

        freights = new ArrayList<>(internalFreights);

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
