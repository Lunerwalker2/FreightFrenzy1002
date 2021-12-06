package org.firstinspires.ftc.teamcode.freightdetection;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class NotebookDemoPipeline extends OpenCvPipeline {




    Mat channel = new Mat();


    @Override
    public Mat processFrame(Mat input){

        Imgproc.cvtColor(input,channel, Imgproc.COLOR_RGB2YCrCb);


        Imgproc.rectangle(input,
                new Rect(
                        (int) (0.25 * input.width()),
                        (int) (0.55 * input.height()),
                        20,
                        20
                ),
                new Scalar(20.0, 220.0, 70.0), 2);

        Imgproc.rectangle(input,
                new Rect(
                        (int) (0.6 * input.width()),
                        (int) (0.55 * input.height()),
                        20,
                        20
                ),
                new Scalar(190.0, 40.0, 70.0), 2);

        Imgproc.putText(channel, "Top level", new Point(170, 30), 0, 0.8, new Scalar(88, 164, 200), 2);


        return input;
    }
}
