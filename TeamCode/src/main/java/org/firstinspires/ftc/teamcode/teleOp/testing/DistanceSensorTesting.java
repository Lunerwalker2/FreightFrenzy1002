package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.util.MB1242;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

@TeleOp
public class DistanceSensorTesting extends CommandOpMode {

    private BNO055IMU imu;
    private double headingOffset = 0;
    private DistanceSensors distanceSensors;
    Pose2d currentPosition = new Pose2d();


    private double getRobotAngle() {
        double raw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        raw -= headingOffset;
        return normalizeRad(raw);
    }

    private static double normalizeRad(double angle) {
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        while (angle >= Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }


    @Override
    public void initialize() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        distanceSensors = new DistanceSensors(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;


        schedule(
                new RelocalizeCommand(
                        (pose) -> currentPosition = new Pose2d(
                                pose.getX(),
                                pose.getY(),
                                pose.getHeading()
                        ),
                        distanceSensors,
                        this::getRobotAngle,
                        false)
        );

    }

    @Override
    public void run() {
        telemetry.addData("Robot X (in)", "%.2f", currentPosition.getX());
        telemetry.addData("Robot Y (in)", currentPosition.getY());
        telemetry.addData("Robot Heading (rad)/(deg)", "%.2f / %.2f", currentPosition.getHeading(), toDegrees(currentPosition.getHeading()));
        telemetry.addData("Forward Sensor MB1242 Range (in)", distanceSensors.getForwardRange(DistanceUnit.INCH));
        telemetry.addData("Left Sensor Rev TOF Range (in)", distanceSensors.getLeftRange(DistanceUnit.INCH));
        telemetry.addData("Right Sensor Rev TOF Range (in)", distanceSensors.getRightRange(DistanceUnit.INCH));
        telemetry.update();
    }
}
