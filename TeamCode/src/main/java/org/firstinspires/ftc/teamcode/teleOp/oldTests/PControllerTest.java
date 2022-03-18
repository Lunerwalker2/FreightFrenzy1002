package org.firstinspires.ftc.teamcode.teleOp.oldTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class PControllerTest extends LinearOpMode {



    private DcMotorEx liftMotor;

    private final double tolerance = 10;
    private final double proportionalValue = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Helps a little against gravity
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        //Set our target of 700 encoder ticks
        double target = 700;

        while (opModeIsActive()){
            //Read the current position of the lift motor
            double currentPosition = liftMotor.getCurrentPosition();

            //Find the error between the current position and the target position
            double error =  target - currentPosition;

            //Find the proportional output, which is just kP * error
            double output = error * proportionalValue;

            //Check if the motor is within the tolerance of the target position
            if(Math.abs(error) > tolerance){
                //If it's not, set the power
                liftMotor.setPower(output);
            } else {
                //If it is, stop the motor
                liftMotor.setPower(0);
            }
        }

    }
}
