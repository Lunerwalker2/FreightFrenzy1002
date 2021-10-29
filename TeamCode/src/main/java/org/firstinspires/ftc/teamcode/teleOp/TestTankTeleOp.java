package org.firstinspires.ftc.teamcode.teleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Test Tape Eater")
public class TestTankTeleOp extends LinearOpMode {


    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    @Override
    public void runOpMode() throws InterruptedException {


        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        double speedMultiplier = 0.8;

        while(opModeIsActive()) {


            if(gamepad1.left_bumper) speedMultiplier = 0.6;
            else speedMultiplier = 0.8;

            double y = cubeInput(-gamepad1.left_stick_y, 0.52); //give more fine control
            double r = gamepad1.right_stick_x;


            leftFront.setPower((y + r) * speedMultiplier);
            leftBack.setPower((y + r) * speedMultiplier);
            rightFront.setPower((y - r) * speedMultiplier);
            rightBack.setPower((y - r) * speedMultiplier);



//            leftFront.setPower(-gamepad1.left_stick_y * speedMultiplier);
//            leftBack.setPower(-gamepad1.left_stick_y * speedMultiplier);
//            rightFront.setPower(gamepad1.right_stick_y * speedMultiplier);
//            rightBack.setPower(gamepad1.right_stick_y * speedMultiplier);

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    //y = ax^3 + x(1-a)
    static double cubeInput(double input, double factor){
        double a = factor * Math.pow(input, 3);
        double b = input * (1 - factor);
        return a + b;
    }

}
