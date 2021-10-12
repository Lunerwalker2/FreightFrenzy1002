package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name="Test Tape Eater")
public class TestTankTeleOp extends LinearOpMode {



    DcMotor left;
    DcMotor right;


    public void runOpMode() throws InterruptedException {


        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();
        while(opModeIsActive()){
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(gamepad1.right_stick_y);
        }

        left.setPower(0);
        right.setPower(0);
    }

}
