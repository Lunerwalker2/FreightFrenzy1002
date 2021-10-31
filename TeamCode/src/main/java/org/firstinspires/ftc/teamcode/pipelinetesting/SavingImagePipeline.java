package org.firstinspires.ftc.teamcode.pipelinetesting;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class SavingImagePipeline extends OpenCvPipeline {
    Mat thingy = new Mat();
    final Object sync = new Object();
    int saveCount = 0;

    @Override
    public Mat processFrame(Mat input)
    {
        synchronized (sync)
        {
            input.copyTo(thingy);
        }

        return input;
    }

    @Override
    public void onViewportTapped()
    {
        synchronized (sync)
        {
            saveMatToDisk(thingy, String.format("EOCV_IMG%d", saveCount));
            saveCount++;
        }
    }
}
