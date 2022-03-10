package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class RedDuckAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rb");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            rightFront.setPower(-1);
            leftFront.setPower(-1);
            rightBack.setPower(-1);
            leftBack.setPower(-1);

            sleep(1000);
        }
    }


}