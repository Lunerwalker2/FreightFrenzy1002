package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SkillIssueTeleOp")
public class SaahasBetterTestTeleOp extends LinearOpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;

    @Override
    public void runOpMode() {
        waitForStart();

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            if (gamepad1.right_stick_y > 0 || gamepad1.right_stick_x > 0) {
                rightFront.setPower(-gamepad1.right_stick_x);
                leftFront.setPower(-gamepad1.right_stick_x);
                rightBack.setPower(gamepad1.right_stick_x);
                leftBack.setPower(gamepad1.right_stick_x);
            }
            else if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_x > 0) {
                rightFront.setPower(-gamepad1.left_stick_y);
                leftFront.setPower(-gamepad1.left_stick_y);
                rightBack.setPower(-gamepad1.left_stick_y);
                leftBack.setPower(-gamepad1.left_stick_y);
            }
            else if ((gamepad1.left_stick_y > 0 || gamepad1.left_stick_x > 0) && (gamepad1.right_stick_y > 0 || gamepad1.right_stick_x > 0)) {
                rightFront.setPower(-gamepad1.left_stick_y);
                leftFront.setPower(-gamepad1.left_stick_y);
                rightBack.setPower(-gamepad1.left_stick_y);
                leftBack.setPower(-gamepad1.left_stick_y);
            }
            else {
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
            }



        }
    }
}
