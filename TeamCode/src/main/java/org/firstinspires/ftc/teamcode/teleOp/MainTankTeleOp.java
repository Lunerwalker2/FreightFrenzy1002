
package org.firstinspires.ftc.teamcode.teleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTankTeleOp extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotor arm;
    DcMotor carousel;

    Servo claw;


    @Override
    public void runOpMode() throws InterruptedException {


        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        arm = hardwareMap.get(DcMotor.class, "arm");

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        
        claw = hardwareMap.get(Servo.class, "claw");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        //        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        boolean wasPressed = false;
        boolean clawOpen = false;

        double speedMultiplier;

        while (opModeIsActive()) {

            if(gamepad2.left_bumper && gamepad2.left_bumper != wasPressed){
                if(!clawOpen){
                    claw.setPosition(0.2);
                    clawOpen = true;
                } else {
                    claw.setPosition(0.63);
                    clawOpen = false;
                }
            }
            wasPressed = gamepad2.left_bumper;
        

            arm.setPower(-gamepad2.left_stick_y);

            if (gamepad2.left_trigger > 0.2) carousel.setPower(0.4);
            else if (gamepad2.right_trigger > 0.2) carousel.setPower(-0.4);
            else carousel.setPower(0);

            //do things lol

            if(gamepad1.left_bumper) speedMultiplier = 0.4;
            else speedMultiplier = 0.7;

            double y = -gamepad1.left_stick_y;  //give more fine control
            double r = gamepad1.right_stick_x;


            leftFront.setPower((y + r) * speedMultiplier);
            leftBack.setPower((y + r) * speedMultiplier);
            rightFront.setPower((y - r) * speedMultiplier);
            rightBack.setPower((y - r) * speedMultiplier);

//            leftFront.setPower(-gamepad1.left_stick_y);
//            leftBack.setPower(-gamepad1.left_stick_y);
//            rightFront.setPower(gamepad1.right_stick_y);
//            rightBack.setPower(gamepad1.right_stick_y);
        }
        
        arm.setPower(0);

        carousel.setPower(0);

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




