package org.firstinspires.ftc.teamcode.teleOp.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicMecTest extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx lb = hardwareMap.get(DcMotorEx.class, "lb");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_y;

            lf.setPower(y + x + rx);
            lb.setPower(y - x + rx);
            rf.setPower(y - x - rx);
            rb.setPower(y + x - rx);
        }
    }
}
