package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "VariableTeleOp")
public class SaahasBetterTestTeleOp extends LinearOpMode {
    DcMotor rightFront = hardwareMap.get(DcMotor.class, "rf");
    DcMotor leftFront = hardwareMap.get(DcMotor.class, "lf");
    DcMotor rightBack = hardwareMap.get(DcMotor.class, "rb");
    DcMotor leftBack = hardwareMap.get(DcMotor.class, "lb");

    @Override
    public void runOpMode() {
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            rightFront.setPower(-gamepad1.left_stick_y);
            leftFront.setPower(-gamepad1.left_stick_y);
            rightBack.setPower(-gamepad1.left_stick_y);
            leftBack.setPower(-gamepad1.left_stick_y);
        }
    }
}
