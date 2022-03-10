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

    public static boolean SLOWMODE;
    public static double rx;
    public static double ry;
    public static double lx;
    public static double ly;
    public static boolean rightBumper;
    public static boolean dPadRight;
    public static boolean dPadLeft;
    public static double COUNT = 0;

    @Override
    public void runOpMode() {
        waitForStart();

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        carouselMotor = hardwareMap.get(DcMotor.class,"carouselMotor");




        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            rightBumper = gamepad1.right_bumper;

            if (rightBumper && (COUNT % 2) == 0) {
                SLOWMODE = true;
                COUNT ++;
            };

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

            while (opModeIsActive()) {
                leftFront.setPower(ly + lx + rx);
                leftBack.setPower(ly - lx + rx);
                rightFront.setPower(ly - lx - rx);
                rightBack.setPower(ly + lx - rx);
            }

            while (opModeIsActive()) {
                if (dPadLeft) {
                    carouselMotor.setPower(-1);
                }
                else if (dPadRight) {
                    carouselMotor.setPower(1);
                }
                else {
                    carouselMotor.setPower(0);
                }
            }



        }
    }
}
