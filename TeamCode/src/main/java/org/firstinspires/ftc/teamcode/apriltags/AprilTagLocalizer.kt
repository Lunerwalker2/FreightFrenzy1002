/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.apriltags

import com.arcrobotics.ftclib.geometry.*
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagPose
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import java.lang.Math.toDegrees
import java.lang.RuntimeException
import java.lang.StringBuilder
import kotlin.math.*

/**
 * Localizer that manages different pipelines and webcams to localization with april tags.
 * Provide a tag size of the tag to detect and its location and rotation on the field.
 *
 * Look at the SDK vuforia examples for how to set the tag position; rotation is in rx, ry, rz.
 * This corresponds tor rx = roll, ry = pitch, and rx = heading/yaw
 *
 * Tag rotation in degrees
 *
 * A tag standing normal to the ground directly facing the pipes in the middle and not twisted at all has
 * x: 0, y: 0, z: 0, yaw: 90, pitch: 0, roll: 90; a tag facing the red alliance would have roll: 0
 *
 * Big thanks to OpenFTC for creating the april tag plugin and also the examples for how to use them.
 * None of this would be possible without their great work.
 *
 *
 * I hate my life.
 */
class AprilTagLocalizer(
        private val tagSize: Double,
        private val tagPosition: Pose,
        vararg webcams: WebcamProfile
) {

    private val pipelineMap = HashMap<WebcamProfile, AprilTagDetectionPipeline>()
    private var localizerState = LocalizerState.NOT_STARTED
    private val openGlTagPose: OpenGLMatrix

    init {
        if (webcams.isNotEmpty()) {//Fill our map with new pipelines for each webcam
            webcams.forEach {
                val pipeline = AprilTagDetectionPipeline(
                        tagSize,
                        it.calibration.fx,
                        it.calibration.fy,
                        it.calibration.cx,
                        it.calibration.cy,
                )
                it.camera.setPipeline(pipeline)
                pipelineMap.put(it, pipeline)
            }
            openGlTagPose = OpenGLMatrix.translation(
                    (tagPosition.x * INCHES_PER_METER).toFloat(),
                    (tagPosition.y.toFloat() * INCHES_PER_METER).toFloat(),
                    (tagPosition.z.toFloat() * INCHES_PER_METER).toFloat()
            ).multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES,
                    tagPosition.roll.toFloat(), //rx
                    tagPosition.pitch.toFloat(), //ry
                    tagPosition.yaw.toFloat() //rz
            ))

        } else {
            throw RuntimeException("Localizer must be given at least 1 camera!")
        }
    }

    /**
     * Runs an update of the localizer. Sees if there is any new detections from the cameras.
     * Returns the average of the camera position from all cameras if there are detections, null otherwise.
     *
     * Distances are returned in inches, rotation in degrees
     */
    fun update(): Pose? {
        //Create a position that will be changed if there are detections and null otherwise
        var cameraPosition: Pose? = null

        if (localizerState == LocalizerState.RUNNING) {
            var isFirstDetectionOfUpdate = true

            /*
        Go through each camera and check for new detection frames. If the list is null then there are
        none. If the list is not null then update the decimation and the current camera position.
         */
            pipelineMap.forEach {
                //Get the list of current detections from the pipeline
                val list: ArrayList<AprilTagDetection>? = it.value.detectionsUpdate

                //Check if it's null
                if (list != null) {
                    if (list.size == 0) {
                        it.key.decimationParameters.numFramesWithoutDetection++

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (it.key.decimationParameters.numFramesWithoutDetection >=
                                it.key.decimationParameters.THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            it.value.setDecimation(it.key.decimationParameters.DECIMATION_LOW)
                        }
                    }
                    //If there is detections
                    else {
                        //Reset the decimation counters
                        it.key.decimationParameters.numFramesWithoutDetection = 0

                        //If the tag is close then raise our decimation to increase performance
                        if (list[0].pose.z < it.key.decimationParameters.THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            it.value.setDecimation(it.key.decimationParameters.DECIMATION_HIGH)

                        }
                        /*
                    Loop through each detection and update the camera position for each.
                    Make sure to set the first one as the camera position because it is still null
                    otherwise, meaning the averaging will fail.
                     */

                        for (detection in list) {

                            //Make a var to hold the given camera pose according to this detection
                            val currentCameraFieldPosition = findCameraPoseFromTag(detection.pose)

                            //Make sure the first run gives the average pose an initial value
                            if (isFirstDetectionOfUpdate) {
                                cameraPosition = currentCameraFieldPosition.copy()
                                isFirstDetectionOfUpdate = false
                            } else {
                                /*Average the camera position for this detection with the current
                                pose using an assertion operator
                                   We do this because kotlin requires a null safe call and this operator means
                                   the program will throw an assertion error on this line if somehow
                                   it is null, which is something we would want to know.
                                 */
                                cameraPosition!!.averageWith(currentCameraFieldPosition)
                            }
                        }
                    }
                }
            }
        }

        //Return our position
        return cameraPosition?.copy()
    }


    /**
     * Adds the given april tag translation to the pose of the tag to find the camera pose.
     *
     * x in our field coordinates correspond to z in the translation if the our field coordinate
     * heading is 0.
     *
     */
    private fun findCameraPoseFromTag(tagTranslation: AprilTagPose): Pose {


        val result: OpenGLMatrix = openGlTagPose.multiplied(
                OpenGLMatrix.translation(
                        (tagTranslation.x * INCHES_PER_METER).toFloat(),
                        (tagTranslation.y * INCHES_PER_METER).toFloat(),
                        (tagTranslation.z * INCHES_PER_METER).toFloat()
                ).multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.RADIANS,
                        tagTranslation.roll.toFloat(),
                        tagTranslation.pitch.toFloat(),
                        tagTranslation.yaw.toFloat()
                )).inverted()
        )

        val translation: VectorF = result.translation
        val rotation: Orientation = Orientation.getOrientation(
                result,
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        )

        /*
        var rangeToTarget: Double = sqrt(tagTranslation.x.pow(2) + tagTranslation.z.pow(2))
        val azimuthToTargetRad: Double = asin(tagTranslation.x / tagTranslation.z)

        rangeToTarget *= (FEET_PER_METER * 12.0) //convert to inches

        //Find the camera's heading on the field by adding the tag heading to the yaw (i think)
        val cameraHeading: Double = normalizeRad(tagPosition.yaw + azimuthToTargetRad)

        val fieldXToTarget: Double = rangeToTarget * cos(cameraHeading)
        val fieldYToTarget: Double = fieldXToTarget / tan(cameraHeading) //find this using x because why not

        //FTCLib vector class
        var translationVec = Vector2d(fieldXToTarget, fieldYToTarget)

        //Now I think we add it to the tag position?
        translationVec = translationVec.plus(Vector2d(tagPosition.x, tagPosition.y))
        */


        return Pose(
                translation[0].toDouble(), //x
                translation[1].toDouble(), //y
                translation[2].toDouble(), //z
                rotation.thirdAngle.toDouble(), //yaw rz
                rotation.secondAngle.toDouble(), //pitch ry
                rotation.firstAngle.toDouble() //roll rx
        )
    }

    /**
     * Allows for pausing the streams if they aren't being used, not required but might help CPU usage?
     *
     * Does nothing if localizer has been stopped or hasn't been started.
     *
     * Once this is called, isPaused will return true and an update will always return null
     * until unPauseLocalizer and/or closeCamerasAndStopStream are called.
     */
    fun pauseLocalizer() {
        if (localizerState == LocalizerState.RUNNING) {
            pipelineMap.forEach {
                it.key.camera.setPipeline(null)
                it.key.camera.pauseViewport()
            }
            localizerState = LocalizerState.PAUSED
        }
    }

    /**
     * Unpauses the stream if it was previously paused. Does nothing if stream was never started or
     * has already stopped.
     */
    fun unPauseLocalizer() {
        if (localizerState == LocalizerState.PAUSED) {
            pipelineMap.forEach {
                it.key.camera.setPipeline(it.value)
                it.key.camera.resumeViewport()
            }
            localizerState = LocalizerState.RUNNING
        }
    }

    /**
     * Opens the cameras async and starts their streams. This is an expensive operation,
     * so be careful. By default this streams in upright.
     */
    fun openCamerasAndStartStream() {
        if (localizerState == LocalizerState.NOT_STARTED) {
            pipelineMap.forEach {
                it.key.camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
                    override fun onOpened() {
                        it.key.camera.startStreaming(it.key.calibration.width, it.key.calibration.height, OpenCvCameraRotation.UPRIGHT)
                    }

                    override fun onError(errorCode: Int) {
                        //Imagine handling your errors couldn't be me
                    }
                })
            }
            localizerState = LocalizerState.RUNNING
        }
    }

    /**
     * Closes all camera devices (non-async) and ends the streams. This is an expensive operation and might take
     * a while to complete.
     */
    fun closeCamerasAndEndStream() {
        if (localizerState == LocalizerState.RUNNING || localizerState == LocalizerState.PAUSED) {
            pipelineMap.forEach {
                it.key.camera.closeCameraDevice()
            }
            localizerState = LocalizerState.STOPPED
        }
    }

    /**
     * Returns a multi-line string that contains performance information about the pipeline(s).
     */
    fun getPipelineTelemetryInfo(): String {
        val builder = StringBuilder()
        var count = 0
        pipelineMap.forEach {
            builder.append("Webcam $count Pipeline FPS: ${it.key.camera.fps} \n")
            builder.append("Webcam $count Overhead ms: ${it.key.camera.overheadTimeMs} \n")
            builder.append("Webcam $count Pipeline ms: ${it.key.camera.pipelineTimeMs} \n")
            count++
        }
        return builder.toString()
    }


    /**
     * Returns true if the pipeline is currently paused (not if it is stopped or not started).
     */
    fun isPaused(): Boolean = localizerState == LocalizerState.PAUSED

    /**
     * Returns true if the pipeline is currently stopped (not if it is paused or not started).
     */
    fun isStopped(): Boolean = localizerState == LocalizerState.STOPPED

    /**
     * Represents the internal state of the localizer
     */
    private enum class LocalizerState {
        NOT_STARTED,
        RUNNING,
        PAUSED,
        STOPPED
    }


    /**
     * Enum for webcam calibs we will use. Fx and fy are focal length; cx and cy are principal point.
     *  Height and width are in pixels.

     */
    enum class WebcamCalibrations(val fx: Double, val fy: Double, val cx: Double, val cy: Double, val width: Int, val height: Int) {
        C270_1280x720(1451.966, 1451.966, 574.203, 356.385, 1280, 720),
        C270_640x480(822.317, 822.317, 319.495, 242.502, 640, 480),
        HD3000_640x480(678.154, 678.17, 318.135, 228.374, 640, 480);
    }

    /**
     * Holds the various values that control the decimation of the pipeline.
     */
    data class DecimationParameters(var numFramesWithoutDetection: Int,
                                    val DECIMATION_HIGH: Float,
                                    val DECIMATION_LOW: Float,
                                    val THRESHOLD_HIGH_DECIMATION_RANGE_METERS: Float,
                                    val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION: Int
    )

    /**
     * Represents a pose on the field of an object.
     * Heading is in radians by default in the (-pi to pi] range
     * Distances are in meters.
     */
    data class Pose(
            var x: Double,
            var y: Double,
            var z: Double,
            var yaw: Double,
            var pitch: Double,
            var roll: Double
    ) {

        /**
         * Sets this position to be the average of the current position and the given one for all
         * components.
         *
         * IN RADIANS
         */
        fun averageWith(nextPose: Pose) {
            x = (x + nextPose.x) / 2.0
            y = (y + nextPose.y) / 2.0
            z = (z + nextPose.z) / 2.0
            yaw = normalizeRad((yaw + nextPose.yaw) / 2.0)
            pitch = normalizeRad((pitch + nextPose.pitch) / 2.0)
            roll = normalizeRad((roll + nextPose.roll) / 2.0)
        }
    }


    /**
     * Class that represents a camera to be added to the detector
     *
     * Each camera added is checked each update and its location is added to the average position
     * for a detection period.
     */
    data class WebcamProfile(val camera: OpenCvCamera,
                             val calibration: WebcamCalibrations,
                             var decimationParameters: DecimationParameters = DecimationParameters(
                                     0,
                                     2.0f,
                                     3.0f,
                                     1.0f,
                                     4
                             )
    )

    companion object {
        const val FEET_PER_METER = 3.28084
        const val INCHES_PER_METER = FEET_PER_METER * 12.0

        /**
         * Normalizes the given angle in degrees to the range of -179 to 180
         */
        fun normalizeDeg(unormalized: Double): Double {
            var angle = unormalized
            while (angle >= 180) angle -= 360.0
            while (angle < -180) angle += 360.0
            return angle
        }

        /**
         * Normalizes the given angle in degrees to the range of (-PI, PI]
         */
        fun normalizeRad(unormalized: Double): Double {
            var angle = unormalized
            while (angle >= Math.PI) angle -= (2.0 * Math.PI)
            while (angle < -Math.PI) angle += (2.0 * Math.PI)
            return angle
        }
    }
}