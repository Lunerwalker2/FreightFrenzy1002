package org.firstinspires.ftc.teamcode.pipelinetesting

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class TeamMarkerPipeline(): OpenCvPipeline() {



    val leftMarkerPositionX: Double = 0.25
    val leftMarkerPositionY: Double = 0.5

    val leftMarkerPositionWidth: Int = 20
    val leftMarkerPositionHeight: Int = 20

    val centerMarkerPositionX: Double = 0.25
    val centerMarkerPositionY: Double = 0.5

    val centerMarkerPositionWidth: Int = 20
    val centerMarkerPositionHeight: Int = 20

    val thresholdValue: Int = 120


    var hubLevel: HubLevel = HubLevel.BOTTOM;


    enum class HubLevel{
        BOTTOM,
        MIDDLE,
        TOP
    }


    //We are going to assume the marker is solid yellow here.

    //Ideally because we can control the color, a solid white or black would be good, or something like that


    var yCrCbMat = Mat()
    var cBMat = Mat()


    override fun processFrame(input: Mat): Mat{

        //Convert to the YCrCb color space from RGB
        Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb)


        //Extract the Cb (blue-difference) channel
        Core.extractChannel(yCrCbMat, cBMat, 2)

        //Make the sample regions
        val leftSampleRect = Rect(
                (leftMarkerPositionX * input.width()).toInt(),
                (leftMarkerPositionY * input.height()).toInt(),
                leftMarkerPositionWidth,
                leftMarkerPositionHeight
        )

        val centerSampleRect = Rect(
                (centerMarkerPositionX * input.width()).toInt(),
                (centerMarkerPositionY * input.height()).toInt(),
                centerMarkerPositionWidth,
                centerMarkerPositionHeight
        )

        //submat our sample regions
        val leftSampleRegion = cBMat.submat(leftSampleRect)
        val centerSampleRegion = cBMat.submat(centerSampleRect)

        val leftRegionMean = Core.mean(leftSampleRegion)
        val centerRegionMean = Core.mean(centerSampleRegion)

        var leftMarkerDetected: Boolean = (leftRegionMean.`val`[0] < thresholdValue)
        var centerMarkerDetected: Boolean = (centerRegionMean.`val`[0] < thresholdValue)

        //if both are detected
        if(leftMarkerDetected && centerMarkerDetected){
            if(leftRegionMean.`val`[0] >= centerRegionMean.`val`[0]){ //include the rare equals case in this because /s
                centerMarkerDetected = false
            } else {
                leftMarkerDetected = false
            }
        }

        //Set what our level is so all can see
        hubLevel = when {
            leftMarkerDetected -> HubLevel.BOTTOM
            centerMarkerDetected -> HubLevel.MIDDLE
            else -> HubLevel.TOP
        }


        //Now that we have all the data we need here, we can start putting things on the viewport for debugging

        //Draw the sample regions

        //Left Region, bottom level
        when(hubLevel){
            HubLevel.TOP, HubLevel.MIDDLE -> Imgproc.rectangle(input, leftSampleRect, Scalar(190.0, 40.0, 70.0), 2)
            HubLevel.BOTTOM -> Imgproc.rectangle(input, leftSampleRect, Scalar(20.0, 220.0, 70.0), 2)
        }

        //Center region, middle level
        when(hubLevel){
            HubLevel.BOTTOM, HubLevel.TOP -> Imgproc.rectangle(input, centerSampleRect, Scalar(190.0, 40.0, 70.0), 2)
            HubLevel.MIDDLE -> Imgproc.rectangle(input, centerSampleRect, Scalar(20.0, 220.0, 70.0), 2)
        }



        //Write some text on the viewport
        Imgproc.putText(input,
            when(hubLevel){
                HubLevel.BOTTOM -> "Bottom Level"
                HubLevel.MIDDLE -> "Middle Level"
                HubLevel.TOP -> "Top Level"
            },
                Point(0.0, 0.0), 1, 1.0, Scalar(255.0, 0.0, 0.0)
        )

        return input;
    }




}