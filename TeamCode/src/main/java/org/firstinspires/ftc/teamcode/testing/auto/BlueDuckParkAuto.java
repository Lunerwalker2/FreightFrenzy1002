package org.firstinspires.ftc.teamcode.testing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueDuckParkAuto")
public class BlueDuckParkAuto extends LinearOpMode {
    DcMotor rightFront, leftFront, rightBack, leftBack, carouselMotor, frontIntake, backIntake, liftMotor;
    Servo backFlap, frontFlap, bucketServo, scoringArmServo;

    @Override
    public void runOpMode() {
        waitForStart();

        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setTargetPosition(100);
        leftFront.setTargetPosition(100);
        rightBack.setTargetPosition(100);
        leftBack.setTargetPosition(100);

    }
}
