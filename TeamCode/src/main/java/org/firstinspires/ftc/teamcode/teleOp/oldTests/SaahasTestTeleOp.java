package org.firstinspires.ftc.teamcode.teleOp.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestTeleOp")
public class SaahasTestTeleOp extends LinearOpMode {
    DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
    DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
    DcMotor rightBack = hardwareMap.get(DcMotor.class, "rb");
    DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");


    @Override
    public void runOpMode() {
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        while (true) {
            if (gamepad1.a) {
                rightFront.setPower(1);
                rightBack.setPower(1);
                leftFront.setPower(1);
                leftBack.setPower(1);
            } else if (gamepad1.b) {
                rightFront.setPower(-1);
                rightBack.setPower(1);
                leftFront.setPower(1);
                leftBack.setPower(-1);
            } else if (gamepad1.x) {
                rightFront.setPower(-1);
                rightBack.setPower(-1);
                leftFront.setPower(-1);
                leftBack.setPower(-1);
            } else if (gamepad1.y) {
                rightFront.setPower(1);
                rightBack.setPower(-1);
                leftFront.setPower(-1);
                leftBack.setPower(1);
            }
        }
    }
}
