package org.firstinspires.ftc.teamcode.teleOp.testing.AarushTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name="Aarush's TeleOp")
public class AarushTeleOp extends LinearOpMode {
    // Declaring the motors to use
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    @Override
    public void runOpMode() {
        // Creates and defines the motors using the hardwareMap
        leftFront = hardwareMap.get(DcMotor.class,"lf");
        rightFront = hardwareMap.get(DcMotor.class,"rf");
        leftBack = hardwareMap.get(DcMotor.class,"lb");
        rightBack = hardwareMap.get(DcMotor.class,"rb");

        // Define arrays of different motors to allow iteration
        DcMotor[] motors = {leftFront,rightFront,leftBack,rightBack};
        DcMotor[] leftMotors = {leftFront,leftBack};
        DcMotor[] rightMotors = {rightFront,rightBack};
        DcMotor[] frontMotors = {leftFront,rightFront};
        DcMotor[] backMotors = {leftBack,rightBack};;

        // Defines basic directions and run modes
        for (DcMotor motor : leftMotors) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Loaded", "Robot is waiting to start!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;
            double power = (y * 0.75) + (x * 0.25);
            setPower(motors, power);
        }
    }
    public void setPower(DcMotor[] motors, double power) {
        if(!((-1.0 <= power)&&(power <= 1.0))) { // If power is not within safe bounds for java
            power = power/100; // Set power as a percentage of the given power
        }
        for (DcMotor motor : motors) motor.setPower(power); // Set the power of all the motors supplied in the method parameters
    }

}
