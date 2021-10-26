
package org.firstinspires.ftc.teamcode.teleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class MainTankTeleOp extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotor arm;
    DcMotor carousel;
    DcMotorSimple intake;


    Servo lol;


    public void runOpMode() throws InterruptedException {


        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        leftFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "rb");

        arm = hardwareMap.get(DcMotor.class, "arm");

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        intake = hardwareMap.get(DcMotorSimple.class, "intake");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            arm.setPower(gamepad2.right_stick_y);

            if (gamepad2.left_trigger > 0.6) carousel.setPower(0.3);
            else if (gamepad2.right_trigger > 0.6) carousel.setPower(-0.3);
            else carousel.setPower(0);

            if (gamepad2.a) intake.setPower(0.7);
            else if (gamepad2.y) intake.setPower(-0.7);
            else intake.setPower(0);

            //do things lol

            leftFront.setPower(-gamepad1.left_stick_y);
            leftBack.setPower(-gamepad1.left_stick_y);
            rightFront.setPower(gamepad1.right_stick_y);
            rightBack.setPower(gamepad1.right_stick_y);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }

}




