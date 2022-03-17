package org.firstinspires.ftc.teamcode.testing.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BluePreloadOneCycleWarehouseParkAuto")
public class BluePreloadOneCycleWarehouseParkAuto extends LinearOpMode {

    //    private boolean SLOWMODE;
//    private double rx, ry, lx, ly;
//    private boolean rightBumper, leftBumper, dPadRight, dPadLeft, DPADUP, DPADDOWN, A, B, X, Y;
//    private double rightTrigger, leftTrigger;
//    private int COUNT = 0;
//
//    DcMotor rightFront, leftFront, rightBack, leftBack, carouselMotor, frontIntake, backIntake, liftMotor;
//    Servo backFlap, frontFlap, bucketServo;
//
    private autonomous auto = new autonomous("blue");

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

//
//        rightFront = hardwareMap.get(DcMotor.class, "rf");
//        leftFront = hardwareMap.get(DcMotor.class, "lf");
//        rightBack = hardwareMap.get(DcMotor.class, "rb");
//        leftBack = hardwareMap.get(DcMotor.class, "lb");
//        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
//        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
//        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
//        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
//        frontFlap = hardwareMap.get(Servo.class, "frontFlap");
//        backFlap = hardwareMap.get(Servo.class, "backFlap");
//        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
//
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        bucketServo.setPosition(0.33);
//
        sleep(1000);
        //TODO: tune values
        auto.driveForward(500);
        auto.liftMotor.setPower(.5);
        sleep(600);
        auto.bucketServo.setPosition(.8);
        sleep(1000);
        auto.bucketServo.setPosition(0.33);
        sleep(1000);
        auto.liftMotor.setPower(-.5);
        sleep(600);
        auto.driveBackward(2000);
        auto.backIntake.setPower(-1);
        sleep(1000);
        auto.backIntake.setPower(1);
        auto.frontIntake.setPower(1);
        sleep(1000);
        auto.driveForward(1000);
        auto.liftMotor.setPower(.5);
        sleep(600);
        auto.bucketServo.setPosition(.8);
        sleep(1000);
        auto.bucketServo.setPosition(0.33);
        sleep(1000);
        auto.liftMotor.setPower(-.5);
        sleep(600);
        auto.driveBackward(1000);
//
//
    }
//
//    public void driveForward(double time) {
//        rightFront.setPower(.5);
//        leftFront.setPower(.5);
//        rightBack.setPower(.5);
//        leftBack.setPower(.5);
//    }
//
//    public void strafeRight(double time) {
//        rightFront.setPower(.5);
//        leftFront.setPower(.5);
//        rightBack.setPower(.5);
//        leftBack.setPower(.5);
//    }
//
//    public void strafeLeft(double time) {
//        rightFront.setPower(.5);
//        leftFront.setPower(.5);
//        rightBack.setPower(.5);
//        leftBack.setPower(.5);
//    }
//
//    public void driveBackward(double time) {
//        rightFront.setPower(.5);
//        leftFront.setPower(.5);
//        rightBack.setPower(.5);
//        leftBack.setPower(.5);
//    }
}