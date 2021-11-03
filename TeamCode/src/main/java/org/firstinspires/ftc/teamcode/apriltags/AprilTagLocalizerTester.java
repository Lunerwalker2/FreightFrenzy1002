package org.firstinspires.ftc.teamcode.apriltags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp(name = "April Tag Localization Test")
public class AprilTagLocalizerTester extends LinearOpMode {


    FtcDashboard dashboard = FtcDashboard.getInstance();

    AprilTagLocalizer localizer;

    OpenCvCamera frontWebcam;
//    OpenCvCamera backWebcam;

    private AprilTagLocalizer.Pose tagPosition = new AprilTagLocalizer.Pose(0, 0, 0, 0, 90, 0);


    @Override
    public void runOpMode() {

        //Dashboard stuff for drawing the robot on the field and for telemetry
        Canvas field;
        TelemetryPacket packet;
        TelemetryPacket initPacket = new TelemetryPacket();



        //In this case we only have 1 webcam so we don't need to split the viewport

        //If we do have to do that then refer to the MultipleCamerasExample in the eocv repo.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        telemetry.addLine("Creating camera objects...");
        telemetry.update();
        initPacket.addLine("Creating camera objects...");
        dashboard.sendTelemetryPacket(initPacket);

        //Create our webcam(s)
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        telemetry.addLine("Creating localizer object...");
        telemetry.update();
        initPacket = new TelemetryPacket();
        initPacket.addLine("Creating localizer object...");
        dashboard.sendTelemetryPacket(initPacket);

        //Create our localizer with the tag position, tag size, and the cameras
        localizer = new AprilTagLocalizer(
                0.096044, //3.78125 in tag
                tagPosition,
                new AprilTagLocalizer.WebcamProfile(
                        frontWebcam,
                        AprilTagLocalizer.WebcamCalibrations.C270_640x480,
                        //If we were in kotlin we wouldn't have to do this as there is a default value, but such is life
                        new AprilTagLocalizer.DecimationParameters(
                                0,
                                2.0f,
                                3.0f,
                                1.0f,
                                4
                        )
                )
        );

        telemetry.addLine("Ready for start");
        telemetry.update();
        initPacket = new TelemetryPacket();
        initPacket.addLine("Ready for start");
        dashboard.sendTelemetryPacket(initPacket);

        waitForStart();

        telemetry.addLine("Starting Cameras...");
        telemetry.update();
        initPacket = new TelemetryPacket();
        initPacket.addLine("Starting Cameras...");
        dashboard.sendTelemetryPacket(initPacket);


        //Start our streams, they will go for the entire program
        localizer.openCamerasAndStartStream();

        /*
        Create the pose object to hold the current position of the robot.

        In an actual auto, you would probably want to run the localizer for a set amount of time,
        and keep the average position of all the calls to update(). However for this tester,
        all we need is the position of the camera throughout the program.
         */
        AprilTagLocalizer.Pose cameraPosition = new AprilTagLocalizer.Pose(0,10,0,0,0, 0);

        if (isStopRequested()) return;

        while (opModeIsActive()){

            packet = new TelemetryPacket();
            field = packet.fieldOverlay();

            packet.addLine(localizer.getPipelineTelemetryInfo());

            //This could return null so we need to check it first
            AprilTagLocalizer.Pose currentPosition = localizer.update();

            //Check if the update returned null (no new position updates)
            if(currentPosition != null) {
                //If it didn't then update the camera's position

                cameraPosition = currentPosition;
            }

            //Print out the positions to telemetry
            packet.put("Camera X", cameraPosition.getX());
            packet.put("Camera Y", cameraPosition.getY());
            packet.put("Camera Yaw", Math.toDegrees(cameraPosition.getYaw()));

            //Draw the robot and tag on the field
            field.setFill("#3F51B5");
            drawRobot(field, new Pose2d(cameraPosition.getX(), cameraPosition.getY(), Math.toDegrees(cameraPosition.getYaw())));


            field.setFill("#00ffb4");
            field.fillRect(tagPosition.getX(), tagPosition.getY(), 4, 4);


            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }

        localizer.closeCamerasAndEndStream();

    }

    //Method copied from DashboardUtil in the rr quickstart for drawing the robot on the dash field
    static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
