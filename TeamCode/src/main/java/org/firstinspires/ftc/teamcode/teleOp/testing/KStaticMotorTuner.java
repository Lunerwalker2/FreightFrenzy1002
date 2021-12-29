package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;


@TeleOp
public class KStaticMotorTuner extends LinearOpMode {

    private double currentPower = 0;

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;


    private boolean prevLF = false;
    private boolean prevLB = false;
    private boolean prevRF = false;
    private boolean prevRB = false;


    ArrayList<DcMotorEx> motors = new ArrayList<>();

    /*
    LF: 0.03253
    LB: 0.03265
    RF: 0.0434
    RB: 0.0425
     */
    @Override
    public void runOpMode() {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");


        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT));

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

//        motors.forEach((motor) -> {motor.setPower(0.3);});
        rb.setPower(0.043);

        waitForStart();

        if (isStopRequested()) return;



        while (opModeIsActive()) {

            telemetry.addData("Current LF", lf.getPower());
            telemetry.addData("Current LB", lb.getPower());
            telemetry.addData("Current RF", rf.getPower());
            telemetry.addData("Current RB", rb.getPower());


            if(gamepad1.a && !prevLF){
                lf.setPower(lf.getPower() + 0.0001);
            }
            if(gamepad1.b && !prevLB){
                lb.setPower(lb.getPower() + 0.0001);
            }
            if(gamepad1.x && !prevRF){
                rf.setPower(rf.getPower() + 0.0001);
            }
            if(gamepad1.y && !prevRB){
                rb.setPower(rb.getPower() + 0.0001);
            }

            prevLF = gamepad1.a;
            prevLB = gamepad1.b;
            prevRF = gamepad1.x;
            prevRB = gamepad1.y;

            if(gamepad1.left_bumper){
                lf.setPower(0);
                lb.setPower(0);
                rf.setPower(0);
                rb.setPower(0);
            }


            telemetry.update();
        }
    }
}
