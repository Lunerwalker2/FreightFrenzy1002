package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Main TeleOp")
public class CheeseTeleOp extends LinearOpMode {

    private DcMotorEx rightFront, leftFront, rightBack, leftBack;
    private BNO055IMU imu;

    private double offset = 0.0;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = orientation.secondAngle; //y axis

            double ly = -gamepad1.left_stick_y;
            double lx = Math.cos(heading) * gamepad1.left_stick_x - Math.sin(heading) * -gamepad1.left_stick_y;
            double rx = Math.sin(heading) * gamepad1.left_stick_x + Math.cos(heading) * -gamepad1.left_stick_y;

            leftFront.setPower(ly + lx + rx);
            leftBack.setPower(ly - lx + rx);
            rightFront.setPower(ly - lx - rx);
            rightBack.setPower(ly + lx - rx);

        }
    }
}
