package org.firstinspires.ftc.teamcode.teleOp.testing;

// Imports the default libraries for the robot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Import the Arrays and Lists so you can be efficient
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Main Test TeleOp")
public class TestTeleOp extends LinearOpMode {
    // Declaring the motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    // Setting power for multiple motors at once
    public void setMassPower(DcMotor[] motors, double power) {
        // Set the power of all the motors to the power assigned
        for (DcMotor motor : motors) motor.setPower(power);
    }
    @Override
    public void runOpMode() {
        // Creates and defines the motors using the hardwareMap
        leftFront = hardwareMap.get(DcMotor.class,"lf");
        rightFront = hardwareMap.get(DcMotor.class,"rf");
        leftBack = hardwareMap.get(DcMotor.class,"lb");
        rightBack = hardwareMap.get(DcMotor.class,"rb");

        List<DcMotor> motors = Arrays.asList(leftFront,rightFront,leftBack,rightBack);
        List<DcMotor> leftMotors = Arrays.asList(leftFront,leftBack);
        List<DcMotor> rightMotors = Arrays.asList(rightFront,rightBack);

        // Defines basic directions and run modes
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Sets the motion of the motor to stop after zero power (may need to remove)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Loaded", "Robot is waiting to start!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // Sets the default power to 1
            double power = 1;
            // Checks for when the bumper is pressed...
            if (gamepad1.left_bumper){
                // ...to activate slow mode with the power
                power = .5;
            }
            // Gets the values from the controllers
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x;
            // Sets the power based on the values given by the controller and math
            leftFront.setPower(power*(y+x));
            leftBack.setPower(power*(y+x));
            rightFront.setPower(power*(y-x));
            rightBack.setPower(power*(y-x));
        }
        // Zeroes the power at the end in order to set the robot into after TeleOp mode.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }


}
