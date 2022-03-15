package org.firstinspires.ftc.teamcode.testing;

/*
███╗░░░███╗░█████╗░███╗░░░███╗███████╗███╗░░██╗████████╗██╗░░░██╗███╗░░░███╗
████╗░████║██╔══██╗████╗░████║██╔════╝████╗░██║╚══██╔══╝██║░░░██║████╗░████║
██╔████╔██║██║░░██║██╔████╔██║█████╗░░██╔██╗██║░░░██║░░░██║░░░██║██╔████╔██║
██║╚██╔╝██║██║░░██║██║╚██╔╝██║██╔══╝░░██║╚████║░░░██║░░░██║░░░██║██║╚██╔╝██║
██║░╚═╝░██║╚█████╔╝██║░╚═╝░██║███████╗██║░╚███║░░░██║░░░╚██████╔╝██║░╚═╝░██║
╚═╝░░░░░╚═╝░╚════╝░╚═╝░░░░░╚═╝╚══════╝╚═╝░░╚══╝░░░╚═╝░░░░╚═════╝░╚═╝░░░░░╚═╝
Authorized for use by FTC 1002 and its members only.
*/


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
            frontFlap, backFlap,
            bucketServo, scoringArmServo;
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
            DUCK_MULTIPLIER = 0.7,
            SCOREVALUE = 0.8,
            STARTVALUE = 0.33;

    BNO055IMU imu;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
            Orientation orientation =
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            angle  = orientation.firstAngle;

            A = gamepad1.a;
            B = gamepad1.b;
            X = gamepad1.x;
            Y = gamepad1.y;

//            Vector2d vector = new Vector2d(
//                    1.1 * (gamepad1.left_stick_x), -(gamepad1.left_stick_y)
//            );

            rx = (gamepad1.right_stick_x);
            ry = -(gamepad1.right_stick_y);
//            lx = vector.rotated(angle).getX();
//            ly = vector.rotated(angle).getY();

            lx = gamepad1.left_stick_x * Math.sin(-angle);
            ly = gamepad1.left_stick_y * Math.cos(-angle);

            dPadRight = gamepad1.dpad_right;
            dPadLeft = gamepad1.dpad_left;
            rightBumper = gamepad1.right_bumper;
            leftBumper = gamepad1.left_bumper;
            rightTrigger = gamepad1.right_trigger;
            leftTrigger = gamepad1.left_trigger;
            dPadUp = gamepad1.dpad_up;
            dPadDown = gamepad1.dpad_down;

            // Run


            //@TODO remove this once works ^^ angle lowest is -1.5, to 1.7 ish


            // Initial Variable sets for run

//            if (toggleRun(A)) {
//                power = 0.5;
//            }
//            else {
//                power = 1;
//            }

            // Runtime
            double leftFrontPower = power * (ly + lx + rx);
            double rightFrontPower = power * (ly - lx + rx);
            double leftBackPower = power * (ly - lx + rx);
            double rightBackPower = power * (ly + lx - rx);

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            if (dPadLeft || dPadRight) {
                int multiplier = 1;
                if (dPadLeft)   multiplier = -1;
                carouselMotor.setPower(multiplier * DUCK_MULTIPLIER);
            } else if (carouselMotor.getPower() != 0) {
                carouselMotor.setPower(0);
            }

            if (rightTrigger > 0) {
                frontIntake.setPower(-1);
                setServoPosition(frontFlap, 1);
            } else if (rightBumper) {
                frontIntake.setPower(1);
                setServoPosition(frontFlap, 1);
            } else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
                setServoPosition(frontFlap, 0);
            }

            if (leftTrigger > 0) {
                backIntake.setPower(-1);
                setServoPosition(backFlap, 1);
            } else if (leftBumper) {
                backIntake.setPower(1);
                setServoPosition(backFlap, 1);
            } else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
                setServoPosition(backFlap, 0);
            }

            if (dPadUp) {
                liftMotor.setPower(.5);
            } else if (dPadDown) {
                liftMotor.setPower(-.5);
            } else {
                liftMotor.setPower(0);
            }

            if (Y) {
                scoringArmServo.setPosition(0.7);
            } else if (A) {
                scoringArmServo.setPosition(0);
            }

            if (X) {
                bucketServo.setPosition(STARTVALUE);
            } else if (B) {
                bucketServo.setPosition(SCOREVALUE);
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
    // Compat functions
        // Functions that make forking from SkillIssue Easier!
    public void setServoPosition (Servo servo, double position) {
        servo.setPosition(position);
    }
}