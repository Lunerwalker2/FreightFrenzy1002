package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.pipeline.TeamMarkerPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class TeamMarkerDetector(private val hardwareMap: HardwareMap) {



    private lateinit var camera: OpenCvCamera
    val teamMarkerPipeline: TeamMarkerPipeline = TeamMarkerPipeline()




    fun initialize() {
        //Get the viewport id
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        //Get the viewport id


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