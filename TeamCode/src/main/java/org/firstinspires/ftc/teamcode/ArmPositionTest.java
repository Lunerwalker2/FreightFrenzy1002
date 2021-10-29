package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
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

        int targetPosition = 51;
        int level = 2;
        while(opModeIsActive()) {

            if(gamepad2.dpad_up && level != 3) level++;
            else if(gamepad2.dpad_down && level != 0) level--;

            switch (level){
                case 0:
                    targetPosition = 1;
                    break;
                case 1:
                    targetPosition = 12;
                    break;
                case 2:
                    targetPosition = 51;
                    break;
                case 3:
                    targetPosition = 71;
                    break;
                default:
                    break;
            }

            double currentPosition = arm.getCurrentPosition();
            arm.setPower(armController.calculate(currentPosition, targetPosition));

            telemetry.addData("Current position", currentPosition);
            telemetry.addData("Target position", targetPosition);
            telemetry.update();
        }



    }
}
