package org.firstinspires.ftc.teamcode.teleOp.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Lift Position Output")
public class LiftPositionOutput extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");



        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
