package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Grater TeleOp")
public class CheeseTeleOp extends LinearOpMode {

    DcMotor rightFront, leftFront, rightBack, leftBack;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = orientation.firstAngle;

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
