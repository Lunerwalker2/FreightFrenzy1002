package org.firstinspires.ftc.teamcode.pipelinetesting

import android.os.Handler
import android.os.HandlerThread
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

@TeleOp
class TeamMarkerSystem(): LinearOpMode() {


    lateinit var camera: OpenCvCamera
    lateinit var pipeline: SavingImagePipeline


    private lateinit var cameraHardwareHandlerThread: HandlerThread
    private lateinit var cameraHardwareHandler: Handler

    var dashboard: FtcDashboard = FtcDashboard.getInstance()

    @Throws(InterruptedException::class)
    override fun runOpMode(){

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)


        camera.setPipeline(pipeline)

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW)
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED)

        cameraHardwareHandlerThread = HandlerThread("CameraHardwareHandlerThread")
        cameraHardwareHandlerThread.start()
        cameraHardwareHandler = Handler(cameraHardwareHandlerThread.looper)


        cameraHardwareHandler.post {
            camera.openCameraDevice()
            camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT)
        }

        waitForStart()

        dashboard.startCameraStream(camera, 30.0)

        while (opModeIsActive()) {
            sleep(20)
        }

        dashboard.stopCameraStream()
        camera.stopStreaming()
        camera.closeCameraDevice()

    }
}