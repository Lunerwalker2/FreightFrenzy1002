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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagPose
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation

/**
 * Localizer that manages different pipelines and webcams to localization with april tags.
 * Provide a tag size of the tag to detect and its location and rotation on the field xy plane.
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
     * Runs an update of the localizer. Sees if there is any new detections from the cameras.
     * Returns the average of the camera position if there are detections, null otherwise.
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


                            //Make a var to hold the given camera translation according to this detection
                            val currentDetectionCameraTranslation = Pose.covertAprilTagPoseToPose(detection.pose)

                            //Translate the given translation by the known tag position
                            currentDetectionCameraTranslation.addTagTo(tagPosition)


                            if (isFirstDetectionOfUpdate) {
                                cameraPosition = currentDetectionCameraTranslation.copy()
                                isFirstDetectionOfUpdate = false
                            } else {

                                /*Average the camera position for this detection with the current
                                pose using an assertion operator
                                   We do this because kotlin requires a null safe call and this operator means
                                   the program will throw an assertion error on this line if somehow
                                   it is null, which is impossible here but you never know.

                                 */
                                cameraPosition!!.averageWith(currentDetectionCameraTranslation)
                            }
                        }
                    }

                }
            }
        }

        //Return our position
        return cameraPosition
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
     * so be careful. By default this streams in 640x480 and upright.
     */
    fun openCamerasAndStartStream() {
        if (localizerState == LocalizerState.NOT_STARTED) {
            pipelineMap.forEach {
                it.key.camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
                    override fun onOpened() {
                        it.key.camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
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
     * Returns true if the pipeline is currently paused (not if it is stopped or not started).
     */
    fun isPaused(): Boolean {
        return localizerState == LocalizerState.PAUSED
    }

    /**
     * Returns true if the pipeline is currently stopped (not if it is paused or not started).
     */
    fun isStopped(): Boolean {
        return localizerState == LocalizerState.STOPPED
    }


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
    Enum for webcam calibs we will use. Fx and fy are focal length; cx and cy are principal point.
     */
    enum class WebcamCalibrations(val fx: Double, val fy: Double, val cx: Double, val cy: Double) {
        C270_1280x720(1451.966, 1451.966, 574.203, 356.385),
        C270_640x480(822.317, 822.317, 319.495, 242.502),
        HD3000_640x480(678.154, 678.17, 318.135, 228.374);
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
         * Adds the given april tag pose to the current pose
         */
        fun addTagTo(pose: Pose){
            x += pose.x
            y += pose.z
            z += pose.y
            yaw = normalizeRad(yaw + pose.yaw)
            pitch = normalizeRad(pitch + pose.pitch)
            roll = normalizeRad(roll + pose.roll)
        }

        /**
         * Adds the given pose to the current pose
         */
        fun addTo(pose: Pose){
            x += pose.x
            y += pose.y
            z += pose.z
            yaw = normalizeRad(yaw + pose.yaw)
            pitch = normalizeRad(pitch + pose.pitch)
            roll = normalizeRad(roll + pose.roll)
        }


        /**
         * Sets this position to be the average of the current position and the given one for all
         * components.
         */
        fun averageWith(nextPose: Pose) {
            x = (x + nextPose.x) / 2.0
            y = (y + nextPose.y) / 2.0
            z = (z + nextPose.z) / 2.0
            yaw = normalizeRad((yaw + nextPose.yaw) / 2.0)
            pitch = normalizeRad((pitch + nextPose.pitch) / 2.0)
            roll = normalizeRad((roll + nextPose.roll) / 2.0)
        }

        companion object {

            /**
             * Converts the april tag pose class of the plugin to the inner pose class
             * found here.
             *
             * The april tag pose has weird axes with the z pointing out and the the x to the right,
             * and y going up and down. This means we have to switch some values around.
             */
            fun covertAprilTagPoseToPose(aprilTagPose: AprilTagPose): Pose {
                //TODO: switch actual vuforia axes to field axes
                return Pose(
                        aprilTagPose.x,
                        aprilTagPose.y,
                        aprilTagPose.z,
                        aprilTagPose.yaw,
                        aprilTagPose.pitch,
                        aprilTagPose.roll
                )
            }
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
                             ))


    companion object {
        const val FEET_PER_METER = 3.28084

        /**
         * Normalizes the given angle in degrees to the range of -179 to 180
         */
        fun normalizeDeg(unormalized: Double): Double {
            var angle = unormalized
            while (angle > 180) angle -= 360.0
            while (angle <= -180) angle += 360.0
            return angle
        }

        /**
         * Normalizes the given angle in degrees to the range of (-PI, PI]
         */
        fun normalizeRad(unormalized: Double): Double {
            var angle = unormalized
            while (angle > Math.PI) angle -= (2.0 * Math.PI)
            while (angle <= -Math.PI) angle += (2.0 * Math.PI)
            return angle
        }
    }
}