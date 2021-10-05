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

package org.firstinspires.ftc.teamcode.AprilTags;

import android.annotation.SuppressLint;
import android.os.Handler;
import android.os.HandlerThread;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import kotlin.collections.MapsKt;

@SuppressLint("DefaultLocale")
@TeleOp
public class AprilTagDemo extends LinearOpMode
{




    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 622.001; //578.272;
    double fy = 622.001; //578.272;
    double cx = 319.803; //402.145;
    double cy = 241.251; //221.506;

    /*
    2560x1440p calib
    fx: 1928.86 fy: 1928.86
    cx: 1294.73 cy: 751.773
     */

    // UNITS ARE METERS
    double tagsize = 0.1721;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private HandlerThread cameraHardwareHandlerThread;
    private Handler cameraHardwareHandler;

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

//         camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//         camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

//         cameraHardwareHandlerThread = new HandlerThread("CameraHardwareHandlerThread");
//         cameraHardwareHandlerThread.start();
//         cameraHardwareHandler = new Handler(cameraHardwareHandlerThread.getLooper());
       camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
       {
           @Override
           public void onOpened()
           {
               camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
           }

           @Override
           public void onError(int errorCode)
           {

           }
       });
//         cameraHardwareHandler.post(new Runnable() {
//             @Override
//             public void run() {
//                 camera.openCameraDevice();
//                 camera.startStreaming(640,480, OpenCvCameraRotation.SIDEWAYS_LEFT);
//             }
//         });


        dashboard.startCameraStream(camera, 30);

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        TelemetryPacket packet;

        double tagX = 0.0;
        double tagY = 0.0;
        double tagYaw = 0.0;

        double robotX = 0.0;
        double robotY = 0.0;
        double robotYaw = 0.0;

        Canvas field;

        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();



            // If there's been a new frame...
            if(detections != null)
            {
                packet = new TelemetryPacket();


                packet.put("FPS", camera.getFps());
                packet.put("Overhead ms", camera.getOverheadTimeMs());
                packet.put("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                    
                        /*
                        This is a terrible way of doing this, but my thought process is that the tag is at 0,0 and facing
                        at wherever the x axis is (0 degrees). We are using -180 > x <= 180 for the range.
                        
                        Theroetically, all we need to do is the add the translation of camera to the position of the tag, and
                        also add the rotation of the camera relative to the tag to the known rotation of the tag, and normalize it.
                        
                        I think.
                        
                        This implementation also only works with one tag, as each recursion o the loop will overwrite the previous
                        */
                        packet.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        packet.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        robotX = tagX + detection.pose.x*FEET_PER_METER;
                        packet.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        robotY = tagY + detection.pose.y*FEET_PER_METER;
                        packet.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        packet.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        robotYaw = normalizeDeg(tagYaw + Math.toDegrees(detection.pose.yaw));
                        packet.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        packet.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }
                /*
                The specific orientation of the tag determines what the axes we need are, but so far I think that the Z axis goes directly out from the middle
                of the tag, and the x axis goes [positive] to the right.
                
                The rotation value we need is aso unknown as of now, but my theory is that it is the yaw.
                
                */
                packet.put("Robot X", robotX);
                packet.put("Robot Y", robotY);
                packet.put("Robot Yaw Deg", robotYaw);

                field = packet.fieldOverlay();
                drawRobot(field, new Pose2d(robotX, robotY, Math.toRadians(robotYaw)));
                dashboard.sendTelemetryPacket(packet);
            }

            sleep(20);
        }
        dashboard.stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    static double normalizeDeg(double angle){
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
