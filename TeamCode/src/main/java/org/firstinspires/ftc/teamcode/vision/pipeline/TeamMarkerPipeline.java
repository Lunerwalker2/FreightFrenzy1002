package org.firstinspires.ftc.teamcode.vision.pipeline;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.function.Function;

public class TeamMarkerPipeline extends OpenCvPipeline {


    public static double leftMarkerPositionX = 0.25;
    public static double leftMarkerPositionY = 0.5;

    public static int leftMarkerPositionWidth = 20;
    public static int leftMarkerPositionHeight = 20;

    public static double centerMarkerPositionX = 0.5;
    public static double centerMarkerPositionY = 0.5;

    public static int centerMarkerPositionWidth = 20;
    public static int centerMarkerPositionHeight = 20;

    public static int thresholdValue = 150;


    //volatile because it's accessed by the opmode thread with no sync
    private volatile HubLevel hubLevel = HubLevel.BOTTOM;



    //We are going to assume the marker is solid yellow here.

    //Ideally because we can control the color, a solid white or black would be good, or something like that


    private Mat yCrCbMat = new Mat();
    private Mat cBMat = new Mat();

    public HubLevel getHubLevel() {
        return hubLevel;
    }

    @Override
    public Mat processFrame(Mat input) {


        //Convert to the YCrCb color space from RGB
        Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);


        //Extract the Cb (blue-difference) channel
        Core.extractChannel(yCrCbMat, cBMat, 2);

        //Make the sample regions
        Rect leftSampleRect = new Rect(
                (int) (leftMarkerPositionX * input.width()),
                (int) (leftMarkerPositionY * input.height()),
                leftMarkerPositionWidth,
                leftMarkerPositionHeight
        );

        Rect centerSampleRect = new Rect(
                (int) (centerMarkerPositionX * input.width()),
                (int) (centerMarkerPositionY * input.height()),
                centerMarkerPositionWidth,
                centerMarkerPositionHeight
        );

        //submat our sample regions
        Mat leftSampleRegion = cBMat.submat(leftSampleRect);
        Mat centerSampleRegion = cBMat.submat(centerSampleRect);

        Scalar leftRegionMean = Core.mean(leftSampleRegion);
        Scalar centerRegionMean = Core.mean(centerSampleRegion);

        boolean leftMarkerDetected = (leftRegionMean.val[0] > thresholdValue);
        boolean centerMarkerDetected = (centerRegionMean.val[0] > thresholdValue);

        //if both are detected
        if (leftMarkerDetected && centerMarkerDetected) {
            if (leftRegionMean.val[0] >= centerRegionMean.val[0]) { //include the rare equals case in this because /s
                centerMarkerDetected = false;
            } else {
                leftMarkerDetected = false;
            }
        }

        //Set what our level is so all can see

        if (leftMarkerDetected) hubLevel = HubLevel.BOTTOM;
        else if (centerMarkerDetected) hubLevel = HubLevel.MIDDLE;
        else hubLevel = HubLevel.TOP;


        //Now that we have all the data we need here, we can start putting things on the viewport for debugging

        //Draw the sample regions

        //Left Region, bottom level
        switch (hubLevel) {
            case TOP:
            case MIDDLE:
                Imgproc.rectangle(input, leftSampleRect, new Scalar(190.0, 40.0, 70.0), 2);
                break;
            case BOTTOM:
                Imgproc.rectangle(input, leftSampleRect, new Scalar(20.0, 220.0, 70.0), 2);
                break;
        }

        //Center region, middle level
        switch (hubLevel) {
            case BOTTOM:
            case TOP:
                Imgproc.rectangle(input, centerSampleRect, new Scalar(190.0, 40.0, 70.0), 2);
                break;
            case MIDDLE:
                Imgproc.rectangle(input, centerSampleRect, new Scalar(20.0, 220.0, 70.0), 2);
                break;
        }


        //Write some text on the viewport
        Imgproc.putText(input,
                ((Function<HubLevel, String>) hublevel -> {
                    switch (hubLevel) {
                        case BOTTOM:
                            return "Bottom Level";
                        case MIDDLE:
                            return "Middle Level";
                        case TOP:
                            return "Top Level";
                        default:
                            return "";
                    }
                }).apply(hubLevel),
                new Point(0.5 * input.width(), 0.2 * input.height()),
                Imgproc.FONT_HERSHEY_COMPLEX,
                1.0,
                new Scalar(255.0, 0.0, 0.0)
        );

        leftSampleRegion.release();
        centerSampleRegion.release();

        return input;
    }

}
