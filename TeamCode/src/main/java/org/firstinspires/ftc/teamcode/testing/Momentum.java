package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "<-$ Momentum $->")
public class Momentum extends LinearOpMode {
    // Momentous Line-Based Optimized Initialization aka M.L.B.O.I.
    DcMotor
            rightFront, leftFront, rightBack, leftBack,
            carouselMotor, frontIntake, backIntake,
            liftMotor;
    Servo
            frontFlap,backFlap,
            bucketServo;
    private double
            rx, ry, lx, ly,
            power = 1,
            angle;
    private float
            rightTrigger, leftTrigger;
    private boolean
            dPadRight, dPadLeft, dPadUp, dPadDown,
            A, B, X, Y,
            rightBumper, leftBumper,
            slowMode = false,
            locked = false;
    private final double
            DUCK_MULTIPLIER = 0.7;

    BNO055IMU imu;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Orientation orientation =
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

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

        waitForStart();

        while (opModeIsActive()) {
            // Initialization of Variables
            angle  = orientation.firstAngle;
            A = gamepad1.a;
            B = gamepad1.b;
            X = gamepad1.x;
            Y = gamepad1.y;

            rx = (gamepad1.right_stick_x);
            ry = -(gamepad1.right_stick_y);
            lx = 1.1 * (gamepad1.left_stick_x);
            ly = -(gamepad1.left_stick_y);

            dPadRight = gamepad1.dpad_right;
            dPadLeft = gamepad1.dpad_left;
            rightBumper = gamepad1.right_bumper;
            leftBumper = gamepad1.left_bumper;
            rightTrigger = gamepad1.right_trigger;
            leftTrigger = gamepad1.left_trigger;
            dPadUp = gamepad1.dpad_up;
            dPadDown = gamepad1.dpad_down;

            // Run

            telemetry.addData("Angle Value", angle);
            telemetry.update();
            //@TODO remove this once works ^^ angle lowest is -1.5, to 1.7 ish


            // Initial Variable sets for run

            if (toggleRun(A)) {
                power = 0.5;
            }
            else {
                power = 1;
            }

            Vector2d g = new Vector2d(lx, ly);

            lx = g.rotated(-angle).getX();
            ly = g.rotated(-angle).getY();

            // Runtime

            leftFront.setPower(power * (ly + lx + rx));
            leftBack.setPower(power * (ly - lx + rx));
            rightFront.setPower(power * (ly - lx - rx));
            rightBack.setPower(power * (ly + lx - rx));

            if (dPadLeft || dPadRight) {
                int multiplier = 1;
                if (dPadLeft) multiplier = -1;
                carouselMotor.setPower(multiplier * DUCK_MULTIPLIER);
            }
            else if (carouselMotor.getPower() != 0) {
                carouselMotor.setPower(0);
            }
            if (rightTrigger > 0) {
                frontIntake.setPower(1);
                frontFlap.setPosition(1);
            }
            else if (rightBumper) {
                frontIntake.setPower(-1);
                frontFlap.setPosition(1);
            }
            else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
                frontFlap.setPosition(0);
            }
            if (leftTrigger > 0) {
                backIntake.setPower(1);
                backFlap.setPosition(1);
            }
            else if (leftBumper) {
                backIntake.setPower(-1);
                backFlap.setPosition(1);
            }
            else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
                backFlap.setPosition(0);

            }

            if (dPadUp) {
                liftMotor.setPower(.5);
            }
            else if (dPadDown) {
                liftMotor.setPower(-.5);
            }
            else {
                liftMotor.setPower(0);
            }

            if (X) {
                bucketServo.setPosition(1);
            }
            else if (B) {
                bucketServo.setPosition(0);
            }
        }
    }
    public boolean toggleRun (boolean condition) {
        if (condition && !locked) {
            locked = true;
            return true;
        }
        if (!condition && locked) {
            locked = false;
        }
        return false;
    }
}