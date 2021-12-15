package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.MB1242;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import java.util.ArrayList;


@TeleOp
public class DistanceSensorTesting extends CommandOpMode {

    private DistanceSensors distanceSensors;
    private SampleMecanumDrive drive;
    private boolean redSide = false;
    private RelocalizeCommand relocalizeCommand;

    private Pose2d distanceSensorPose = new Pose2d();

    @Override
    public void initialize() {

        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-33.6, 64.0, toRadians(-90.0)));

        distanceSensors = new DistanceSensors(hardwareMap);

        relocalizeCommand = new RelocalizeCommand(
                (pose) -> {
                    //Do the useless average position update
                    distanceSensorPose = new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        pose.getHeading()
                    );
                    //More importantly, update the current pose estimate for live debugging
                    drive.setPoseEstimate(new Pose2d(
                            relocalizeCommand.lastInstantPosition.getX(),
                            relocalizeCommand.lastInstantPosition.getY(),
                            relocalizeCommand.lastInstantPosition.getHeading()
                    ));
                },
                distanceSensors,
                drive::getExternalHeading,
                redSide
        );
        schedule(relocalizeCommand);

    }


    @Override
    public void run() {


        //Send our joystick values to the rr drive class.
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        //Update the motor powers in rr and the rr localizer

        //Use the debugging instant position
        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addLine("Red side"+redSide);
        telemetry.addData("Robot X (in)", "%.3f", poseEstimate.getX());
        telemetry.addData("Robot Y (in)", "%.3f", poseEstimate.getY());
        telemetry.addData("Robot Heading (rad)/(deg)", "%.3f / %.3f",
                poseEstimate.getHeading(), toDegrees(poseEstimate.getHeading()));
        telemetry.addData("Distance Sensor Cycle Time (ms)", distanceSensors.getCycleTime());
        telemetry.addData("Forward Sensor MB1242 Range (in)", distanceSensors.getForwardRange(DistanceUnit.INCH));
        telemetry.addData("Left Sensor Rev TOF Range (in)", distanceSensors.getLeftRange(DistanceUnit.INCH));
        telemetry.addData("Right Sensor Rev TOF Range (in)", distanceSensors.getRightRange(DistanceUnit.INCH));

        telemetry.update();


    }
}
