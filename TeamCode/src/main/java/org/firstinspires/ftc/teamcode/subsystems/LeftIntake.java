package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;

public class LeftIntake extends SubsystemBase implements IntakeSide {

    private final DcMotorEx intakeMotor;
    private final Servo armServo;
//    private final Rev2mDistanceSensor intakeSensor;

    public boolean up = true;

    public LeftIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armServo = hardwareMap.get(Servo.class, "leftIntakeArm");
//        intakeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftIntakeSensor");
        intakeUp();
    }

    @Override
    public void periodic(){

    }

    public void intake(){
        intakeMotor.setPower(0.5);
    }

    public void stop(){
        intakeMotor.setPower(0);
    }

    public void outtake(){
        intakeMotor.setPower(-0.3);
    }

    public void intakePower(double power){
        intakeMotor.setPower(power);
    }

    public void intakeDown(){
        armServo.setPosition(0.0);
        up = false;
    }

    public void intakeUp(){
        armServo.setPosition(1.0);
        up = true;
    }

//    @Override
//    public boolean freightDetected(){
//        return intakeSensor.getDistance(DistanceUnit.INCH) < 2.0;
//    }
    public int currentPosition(){
        return intakeMotor.getCurrentPosition();
    }
}
