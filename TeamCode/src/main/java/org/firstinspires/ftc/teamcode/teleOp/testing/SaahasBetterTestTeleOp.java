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

            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;
            double lx = gamepad1.left_stick_x * 1.1;
            double ly = -gamepad1.left_stick_y;

            leftFront.setPower(ly + lx + rx);
            leftBack.setPower(ly - lx + rx);
            rightFront.setPower(ly - lx - rx);
            rightBack.setPower(ly + lx - rx);





        }
    }
}
