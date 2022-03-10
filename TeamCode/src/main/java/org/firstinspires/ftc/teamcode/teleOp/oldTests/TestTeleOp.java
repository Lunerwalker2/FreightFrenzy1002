package org.firstinspires.ftc.teamcode.teleOp.oldTests;

// Imports the default libraries for the robot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Import the Arrays and Lists so you can be efficient
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Balanced TeleOp")
public class TestTeleOp extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        waitForStart();

        leftFront = hardwareMap.get(DcMotor.class,"lf");
        rightFront = hardwareMap.get(DcMotor.class,"rf");
        leftBack = hardwareMap.get(DcMotor.class,"lb");
        rightBack = hardwareMap.get(DcMotor.class,"rb");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        double power = 1, leftX = 0, leftY = 0, rightX = 0;
        while (opModeIsActive()) {
            power = 1;
            leftY = -gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;
            leftX = gamepad1.left_stick_x;

            if (gamepad1.left_bumper){
                power = .5;
                telemetry.addData("Activated", "Slow mode");
            }
            if (!((leftX >= -0.1)&&(leftX <= 0.1))) {
                leftFront.setPower(1*power*leftX);
                leftBack.setPower(-1*power*leftX);
                rightFront.setPower(-1*power*leftX);
                rightBack.setPower(1*power*leftX);
            }
            else {
                double rightPower = power*(leftY-rightX);
                double leftPower = power*(leftY+rightX);
                leftFront.setPower(leftPower);
                leftBack.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightBack.setPower(rightPower);
            }

        }
    }


}
