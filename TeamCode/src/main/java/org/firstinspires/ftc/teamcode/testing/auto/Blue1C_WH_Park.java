package org.firstinspires.ftc.teamcode.testing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Blue Warehouse 1-Cycle")
public class Blue1C_WH_Park extends LinearOpMode {
    DcMotor
            rightFront, leftFront, rightBack, leftBack;
    DcMotor[] wheels = {rightFront, leftFront, rightBack, leftBack};


    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 15.36;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : wheels) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        waitForStart();
        encoderDrive(0.6, 20, 48);

        sleep(1000);     // pause for servos to move
    }

    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int lf, rf, rb, lb;

        if (opModeIsActive()) {
            lf = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            rf = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            lb = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            rb = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(lf);
            rightFront.setTargetPosition(rf);
            leftBack.setTargetPosition(lb);
            rightBack.setTargetPosition(rb);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));

            leftFront.setPower(0);
            rightFront.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}