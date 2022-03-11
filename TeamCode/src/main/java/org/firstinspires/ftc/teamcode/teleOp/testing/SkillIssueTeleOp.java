package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SkillIssueTeleOp")
public class SkillIssueTeleOp extends LinearOpMode {
    DcMotor rightFront,leftFront,rightBack,leftBack,carouselMotor,frontIntake,backIntake, liftMotor;
    Servo backFlap,frontFlap,bucketServo;

    //initialize
    private boolean SLOWMODE;
    private double rx,ry,lx,ly;
    private boolean rightBumper, leftBumper,dPadRight,dPadLeft,DPADUP,DPADDOWN,A,B,X,Y;
    private double rightTrigger, leftTrigger;
    private int COUNT = 0;

    @Override
    public void runOpMode() {
        //drivers pick up your controllers
        waitForStart();

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        carouselMotor = hardwareMap.get(DcMotor.class,"carouselMotor");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        frontFlap = hardwareMap.get(Servo.class, "frontFlap");
        backFlap = hardwareMap.get(Servo.class, "backFlap");
        bucketServo = hardwareMap.get(Servo.class,"bucketServo");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        bucketServo.setPosition(0);

        while (opModeIsActive()) {

            A = gamepad1.a;
            B = gamepad1.b;
            X = gamepad1.x;
            Y = gamepad1.y;
            dPadRight = gamepad1.dpad_right;
            dPadLeft = gamepad1.dpad_left;
            rightBumper = gamepad1.right_bumper;
            leftBumper = gamepad1.left_bumper;
            rightTrigger = gamepad1.right_trigger;
            leftTrigger = gamepad1.left_trigger;
            DPADUP = gamepad1.dpad_up;
            DPADDOWN = gamepad1.dpad_down;


            if (A && (COUNT % 2) == 1) {
                SLOWMODE = true;
                COUNT ++;
            }
            // slow mode but bad
            //TODO make good-er
            if (SLOWMODE) {
                rx = gamepad1.right_stick_x * .5;
                ry = -gamepad1.right_stick_y * .5;
                lx = gamepad1.left_stick_x * 1.1 * .5;
                ly = -gamepad1.left_stick_y * .5;
            }

            else {
                rx = gamepad1.right_stick_x;
                ry = -gamepad1.right_stick_y;
                lx = gamepad1.left_stick_x * 1.1;
                ly = -gamepad1.left_stick_y;
            }

            if (opModeIsActive()) {
                leftFront.setPower(ly + lx + rx);
                leftBack.setPower(ly - lx + rx);
                rightFront.setPower(ly - lx - rx);
                rightBack.setPower(ly + lx - rx);
            }

            //Ducks go brrr
            if (opModeIsActive()) {
                if (dPadLeft) {
                    carouselMotor.setPower(-.7);
                }
                else if (dPadRight) {
                    carouselMotor.setPower(.7);
                }
                else {
                    carouselMotor.setPower(0);
                }
            }

            if (opModeIsActive()) {
                if (rightTrigger > 0) {
                    frontIntake.setPower(-1);
                    setServoPosition(frontFlap, 1);
                }
                else if (rightBumper) {
                    frontIntake.setPower(1);
                    setServoPosition(frontFlap, 1);
                }
                else {
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    setServoPosition(frontFlap, 0);
                }
                if (leftTrigger > 0) {
                    backIntake.setPower(-1);
                    setServoPosition(backFlap, 1);
                }
                else if (leftBumper) {
                    backIntake.setPower(1);
                    setServoPosition(backFlap, 1);
                }
                else {
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    setServoPosition(backFlap, 0);
                }
            }

            if (opModeIsActive()) {
                if (DPADUP) {
                    liftMotor.setPower(.5);
                }
                else if (DPADDOWN) {
                    liftMotor.setPower(-.5);
                }
                else {
                    liftMotor.setPower(0);
                }
            }

            if (opModeIsActive()) {
                if (X) {
                    bucketServo.setPosition(1);
                }
                else if (B) {
                    bucketServo.setPosition(0);
                }
            }



        }
    }
    public void setServoPosition(Servo servo, double position) {
        servo.setPosition(position);
    }
}
