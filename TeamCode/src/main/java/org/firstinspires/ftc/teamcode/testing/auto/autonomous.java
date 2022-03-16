package org.firstinspires.ftc.teamcode.testing.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class autonomous {

    private boolean slowMode;
    private double rx, ry, lx, ly;
    private boolean rightBumper, leftBumper, dPadRight, dPadLeft, DPADUP, DPADDOWN, A, B, X, Y;
    private double rightTrigger, leftTrigger;
    private int COUNT = 0;
    private boolean isRed;

    DcMotor rightFront, leftFront, rightBack, leftBack, carouselMotor, frontIntake, backIntake, liftMotor;
    Servo backFlap, frontFlap, bucketServo;

    public autonomous(String allianceIn) {
        if (allianceIn.equals("red")) {
            isRed = true;
        } else if (allianceIn.equals("blue")) {
            isRed = false;
        }
    }

    public void initialize() {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        frontFlap = hardwareMap.get(Servo.class, "frontFlap");
        backFlap = hardwareMap.get(Servo.class, "backFlap");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketServo.setPosition(0.33);

    }

    public void driveForward(int time) throws InterruptedException {
        rightFront.setPower(.5);
        leftFront.setPower(.5);
        rightBack.setPower(.5);
        leftBack.setPower(.5);
        sleep(time);
    }

    public void strafeRight(int time) throws InterruptedException {
        rightFront.setPower(.5);
        leftFront.setPower(.5);
        rightBack.setPower(.5);
        leftBack.setPower(.5);
        sleep(time);
    }

    public void strafeLeft(int time) throws InterruptedException {
        rightFront.setPower(.5);
        leftFront.setPower(.5);
        rightBack.setPower(.5);
        leftBack.setPower(.5);
        sleep(time);
    }

    public void driveBackward(int time) throws InterruptedException {
        rightFront.setPower(.5);
        leftFront.setPower(.5);
        rightBack.setPower(.5);
        leftBack.setPower(.5);
        sleep(time);
    }
    public void reset() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }
}