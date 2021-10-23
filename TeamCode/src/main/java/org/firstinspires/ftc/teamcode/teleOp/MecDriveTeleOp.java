package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;

@TeleOp(name="Simple Mecnaum Drive")
public class MecDriveTeleOp extends LinearOpMode {


    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;


    static final double VX_WEIGHT = 1;
    static final double VY_WEIGHT = 1;
    static final double OMEGA_WEIGHT = 1;

    double slowModeMult = 1.0;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");


        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }); //Sets the power decrease of RUE to 0%, making the max speed back to 100%

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()){
            if(isStopRequested()) return;

            double y = (Math.abs(gamepad1.left_stick_y) > 0.05) ? cubeInput(-gamepad1.left_stick_y, 0.52) : 0.0; // Remember, this is reversed!
            double x = (Math.abs(gamepad1.left_stick_x) > 0.05) ? cubeInput(gamepad1.left_stick_x * 1.1, 0.52) : 0.0; // Counteract imperfect strafing
            double rx = (Math.abs(gamepad1.right_stick_x) > 0.05) ? cubeInput(gamepad1.right_stick_x, 0.52) : 0.0;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            slowModeMult = gamepad1.left_bumper ? 0.5 : 1; //If the left bumper is pressed, reduce the speed


            lf.setPower(frontLeftPower * slowModeMult);
            lb.setPower(backLeftPower * slowModeMult);
            rf.setPower(frontRightPower * slowModeMult);
            rb.setPower(backRightPower * slowModeMult);

        }


    }

    double cubeInput(double input, double factor){
        double t = factor * Math.pow(input, 3);
        double r = input * (1-factor);
        return t+r;
    }
}
