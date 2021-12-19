package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Extensions;

@Disabled
@TeleOp
public class HeadingSaverTest extends LinearOpMode {



    BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException{
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);


        double currentHeading = 0;
        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){
            currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        }


        Extensions.HEADING_SAVER = currentHeading;


    }
}
