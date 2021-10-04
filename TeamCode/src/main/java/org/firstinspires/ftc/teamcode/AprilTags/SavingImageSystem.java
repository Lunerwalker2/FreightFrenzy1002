package org.firstinspires.ftc.teamcode.AprilTags;

import android.os.Handler;
import android.os.HandlerThread;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class SavingImageSystem extends LinearOpMode {


    OpenCvCamera camera;
    SavingImagePipeline pipeline;



    private HandlerThread cameraHardwareHandlerThread;
    private Handler cameraHardwareHandler;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        camera.setPipeline(pipeline);

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        cameraHardwareHandlerThread = new HandlerThread("CameraHardwareHandlerThread");
        cameraHardwareHandlerThread.start();
        cameraHardwareHandler = new Handler(cameraHardwareHandlerThread.getLooper());


        cameraHardwareHandler.post(new Runnable() {
            @Override
            public void run() {
                camera.openCameraDevice();
                camera.startStreaming(640,480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        dashboard.startCameraStream(camera, 30);

        while (opModeIsActive()){

        }

        dashboard.stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();


    }
}
