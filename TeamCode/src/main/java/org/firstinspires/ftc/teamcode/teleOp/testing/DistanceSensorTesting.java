package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.MB1242;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import java.util.ArrayList;

@TeleOp
public class DistanceSensorTesting extends CommandOpMode {

    private BNO055IMU imu;
    private double headingOffset = 0;
    private DistanceSensors distanceSensors;
    Pose2d currentPosition = new Pose2d();

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    private double getRobotAngle() {
        double raw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        return AngleUnit.normalizeRadians(raw - headingOffset);
    }


    @Override
    public void initialize() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }); //Sets the power decrease of RUE to 0%, making the max speed back to 100%

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

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


        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);

        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawRobot(packet.fieldOverlay(), currentPosition);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }
}
