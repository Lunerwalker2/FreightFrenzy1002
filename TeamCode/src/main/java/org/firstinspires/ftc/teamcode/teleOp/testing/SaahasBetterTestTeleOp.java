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
    DcMotor carouselMotor;
    DcMotor frontIntake;
    DcMotor backIntake;

    //initialize
    public static boolean SLOWMODE;
    public static double rx;
    public static double ry;
    public static double lx;
    public static double ly;
    public static boolean rightBumper;
    public static boolean leftBumper;
    public static float rightTrigger;
    public static float leftTrigger;
    public static boolean dPadRight;
    public static boolean dPadLeft;
    public static int COUNT = 0;
    public static boolean DPADUP;
    public static boolean DPADDOWN;
    public static boolean A;
    public static boolean B;
    public static boolean X;
    public static boolean Y;

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




        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            A = gamepad1.a;
            dPadRight = gamepad1.dpad_right;
            dPadLeft = gamepad1.dpad_left;
            rightBumper = gamepad1.right_bumper;
            leftBumper = gamepad1.left_bumper;
            rightTrigger = gamepad1.right_trigger;
            leftTrigger = gamepad1.left_trigger;


            if (A && (COUNT % 2) == 1) {
                SLOWMODE = true;
                COUNT ++;
            }
            // slow mode but bad
            //TODO make good
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
                    frontIntake.setPower(1);
                }
                else if (rightBumper) {
                    frontIntake.setPower(-1);
                }
                else {
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                }
                if (leftTrigger > 0) {
                    backIntake.setPower(1);
                }
                else if (leftBumper) {
                    backIntake.setPower(-1);
                }
                else {
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                }
            }



        }
    }
}
