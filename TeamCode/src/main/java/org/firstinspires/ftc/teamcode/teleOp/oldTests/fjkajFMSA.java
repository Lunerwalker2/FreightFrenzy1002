package org.firstinspires.ftc.teamcode.teleOp.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "testAuto")
public class fjkajFMSA extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rb");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        sleep(5000);

        rightFront.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(1);
        leftBack.setPower(1);

        sleep(1000);

        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);


    }
}
