package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Extensions;

import java.util.ArrayList;


//@Disabled
@TeleOp
public class FieldRelativeDrive extends LinearOpMode {

    boolean prevState = false;

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;

    BNO055IMU imu;
    double offset = 0;

    double slowModeMult = 1.0;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    double getRobotAngle() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        angle = AngleUnit.normalizeRadians(angle - offset);
        return angle;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        offset = Extensions.HEADING_SAVER;


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

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


        while (!isStarted()) {
            telemetry.addLine("Current offset is " + offset);
            telemetry.addLine("Press left bumper to reset to current heading");
            telemetry.update();

            if (gamepad1.left_bumper) offset = getRobotAngle();
        }


        while (opModeIsActive()) {
            if (isStopRequested()) return;

            double heading = getRobotAngle();


            if (gamepad1.a && !prevState) offset += heading;
            prevState = gamepad1.a;

            double y = (Math.abs(gamepad1.left_stick_y) > 0.05) ? cubeInput(-gamepad1.left_stick_y, 0.4) : 0.0; // Remember, this is reversed!
            double x = (Math.abs(gamepad1.left_stick_x) > 0.05) ? cubeInput(gamepad1.left_stick_x * 1.1, 0.4) : 0.0; // Counteract imperfect strafing
            double rx = (Math.abs(gamepad1.right_stick_x) > 0.05) ? cubeInput(gamepad1.right_stick_x, 0.4) : 0.0;

            //Get the field centric inputs

            Pose2d pose = Extensions.Companion.toFieldRelative(new Pose2d(x, y, rx), heading);

            x = pose.getX();
            y = pose.getY();
            rx = pose.getHeading();

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            slowModeMult = gamepad1.left_bumper ? 0.5 : 1; //If the left bumper is pressed, reduce the speed


            lf.setPower(frontLeftPower * slowModeMult);
            lb.setPower(backLeftPower * slowModeMult);
            rf.setPower(frontRightPower * slowModeMult);
            rb.setPower(backRightPower * slowModeMult);

            telemetry.addLine("Press the left bumper to re-zero the heading.");
            telemetry.addData("Current Heading with offset", AngleUnit.DEGREES.fromRadians(getRobotAngle()));
            telemetry.addData("Offset", AngleUnit.DEGREES.fromRadians(offset));
            telemetry.update();

        }


    }

    double cubeInput(double input, double factor) {
        double t = factor * Math.pow(input, 3);
        double r = input * (1 - factor);
        return t + r;
    }
}
