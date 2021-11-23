package org.firstinspires.ftc.teamcode.freightdetection;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class NotebookDemoPipeline extends OpenCvPipeline {




    Mat channel = new Mat();


    @Override
    public Mat processFrame(Mat input){

        Imgproc.cvtColor(input,channel, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(channel, channel, 2);

        Imgproc.putText(channel, "Cb channel", new Point(170, 30), 0, 0.8, new Scalar(88, 164, 200), 2);


        return channel;
    }
}
