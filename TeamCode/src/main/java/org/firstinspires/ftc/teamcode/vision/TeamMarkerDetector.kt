package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.pipeline.TeamMarkerPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

/*
Class that abstracts our vision pipeline and all the EOCV things needed to run it.

The constructor can be called at any time, as long as the init() function is called
inside the opmode (once the hardware map is set).

startStream() is called once you want the pipeline to start, likely at the end of the init stuff
and right before the loop that reads the hub level.

the getHubLevel() in the pipeline class can be called to get the current hub level, this should
probably be called until the end of init to get the last detection after randomization.

endStream() should be called after init, at start. It is blocking so it might have a slight delay.
 */
class TeamMarkerDetector(private val hardwareMap: HardwareMap) {



    private lateinit var camera: OpenCvCamera
    val teamMarkerPipeline: TeamMarkerPipeline = TeamMarkerPipeline()




    fun init() {
        //Get the viewport id
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        camera.setPipeline(teamMarkerPipeline)

    }

    fun setLeftRectangle(x: Double, y: Double){
        TeamMarkerPipeline.leftMarkerPositionX = x
        TeamMarkerPipeline.leftMarkerPositionY = y
    }

    fun setRightRectangle(x: Double, y: Double){
        TeamMarkerPipeline.centerMarkerPositionX = x
        TeamMarkerPipeline.centerMarkerPositionY = y
    }

    fun setRectangleSize(width: Int, height: Int){
        TeamMarkerPipeline.leftMarkerPositionWidth = width
        TeamMarkerPipeline.leftMarkerPositionHeight = height
        TeamMarkerPipeline.centerMarkerPositionWidth = width
        TeamMarkerPipeline.centerMarkerPositionHeight = height
    }


    fun startStream(){
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {

            }
        })
    }

    fun endStream(){
        camera.stopStreaming()
        camera.closeCameraDevice()
    }

}