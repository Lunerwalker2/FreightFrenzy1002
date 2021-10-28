package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class ArmPositionTest extends LinearOpMode {



    DcMotorEx arm;


    @Override
    public void runOpMode(){


        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPositionPIDFCoefficients(5);

        waitForStart();

        sleep(1000);

        arm.setTargetPosition(51);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.15);

        arm.setTargetPositionTolerance(9);

        while(opModeIsActive()) {

            if(!arm.isBusy()) arm.setPower(0.5);

            telemetry.addData("current position", arm.getCurrentPosition());
            telemetry.update();
        }



    }
}
