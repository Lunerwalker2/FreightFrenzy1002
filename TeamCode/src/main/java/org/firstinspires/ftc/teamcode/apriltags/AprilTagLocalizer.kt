package org.firstinspires.ftc.teamcode.apriltags

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

/**
 * Localizer that manages different pipelines and webcams to localization with april tags.
 * Provide a tag size of the tag to detect.
 */
class AprilTagLocalizer(
        private val tagSize: Double,
        vararg webcams: WebcamProfile
) {

    companion object {
        const val FEET_PER_METER = 3.28084
    }

    var currentPosition = RobotPose(0.0,0.0,0.0)


    var numFramesWithoutDetection = 0

    val DECIMATION_HIGH = 3f
    val DECIMATION_LOW = 2f
    val THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f
    val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4

    private val pipelineMap = HashMap<WebcamProfile, AprilTagDetectionPipeline>()

    init {
        //Fill our map with new pipelines for each webcam
        webcams.forEach {
            pipelineMap[it] = AprilTagDetectionPipeline(
                    tagSize,
                    it.calibration.fx,
                    it.calibration.fy,
                    it.calibration.cx,
                    it.calibration.cy,
            )
        }
    }


    /**
     * Opens the cameras async and starts their streams. This is an expensive operation,
     * so be careful. By default this streams in 640x480 and upright.
     */
    fun openCamerasAndStartStream(){
        pipelineMap.forEach {
            it.key.camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
                override fun onOpened() {
                    it.key.camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                }

                override fun onError(errorCode: Int) {
                    TODO("Not yet implemented")
                }
            })
        }
    }


    fun closeCamerasAndEndStream(){
        pipelineMap.forEach {
            it.key.camera.closeCameraDevice()
        }
    }




    /**
     * Represents the robot's position on the field in the traditional ftc field coordinates.
     * Heading is in radians by default although headingAsDeg() can help with degrees. Both are in
     * euler angles.
     */
    data class RobotPose(var x: Double, var y: Double, var r: Double) {
        /**
         * Sets this position to be the average of the current position and the given one for all
         * components.
         */
        fun averageWith(nextPose: RobotPose) {
            x = (x + nextPose.x) / 2.0
            y = (y + nextPose.y) / 2.0
            r = (r + nextPose.r) / 2.0
        }

        /**
         * Returns the heading value in degrees.
         */
        fun headingAsDeg(): Double {
            return Math.toDegrees(r);
        }
    }


    /**
     * Class that represents a camera to be added to the detector
     *
     * Each camera added is checked each update and its location is added to the average position
     * for a detection period.
     */
    data class WebcamProfile(val camera: OpenCvCamera, val calibration: WebcamCalibrations)

    /**
    Enum for webcam calibs we will use. Fx and fy are focal length; cx and cy are principal point.
     */
    enum class WebcamCalibrations(val fx: Double, val fy: Double, val cx: Double, val cy: Double) {
        C270_1280x720(1451.966, 1451.966, 574.203, 356.385),
        C270_640x480(822.317, 822.317, 319.495, 242.502),
        HD3000_640x480(678.154, 678.17, 318.135, 228.374);
    }
}