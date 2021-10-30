package org.firstinspires.ftc.teamcode.teleOp;


import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class ArmTestTeleOp extends LinearOpMode {


    DcMotorEx arm;


    @Override
    public void runOpMode(){


        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        sleep(1000);



        while(opModeIsActive()) {

            arm.setPower(-gamepad2.left_stick_y);

            telemetry.addData("arm current amps: ", arm.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }



    }
}
