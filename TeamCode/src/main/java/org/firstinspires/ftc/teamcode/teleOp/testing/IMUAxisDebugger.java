package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class IMUAxisDebugger extends LinearOpMode {

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){

            Orientation orientation =
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            AngularVelocity angularVelocity = imu.getAngularVelocity();

            telemetry.addData("Orientation X Deg", "%.3f", orientation.firstAngle);
            telemetry.addData("Orientation Y Deg", "%.3f", orientation.secondAngle);
            telemetry.addData("Orientation Z Deg", "%.3f", orientation.thirdAngle);

            telemetry.addData("Angular Velocity X Deg/s", "%.3f", angularVelocity.xRotationRate);
            telemetry.addData("Angular Velocity Y Deg/s", "%.3f", angularVelocity.yRotationRate);
            telemetry.addData("Angular Velocity Z Deg/s", "%.3f", angularVelocity.zRotationRate);

            telemetry.update();
        }
    }
}
