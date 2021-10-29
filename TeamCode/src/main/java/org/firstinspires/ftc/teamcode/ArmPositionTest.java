package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class ArmPositionTest extends LinearOpMode {



    PIDFController armController;
    DcMotorEx arm;


    @Override
    public void runOpMode(){

        armController = new PIDFController(5, 0, 0, 0.05, 0, 0);
        armController.setTolerance(5);


        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        sleep(1000);

        armController.setSetPoint(51);

        while(opModeIsActive()) {
            double currentPosition = arm.getCurrentPosition();
            arm.setPower(armController.calculate(currentPosition));

            telemetry.addData("current position", currentPosition);
            telemetry.update();
        }



    }
}
