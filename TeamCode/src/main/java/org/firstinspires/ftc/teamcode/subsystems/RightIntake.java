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

public class RightIntake extends SubsystemBase implements IntakeSide {

    private final DcMotorEx intakeMotor;
    private final Servo armServo;
//    private final Rev2mDistanceSensor intakeSensor;

    public boolean up = true;

    public RightIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armServo = hardwareMap.get(Servo.class, "rightIntakeArm");
//        intakeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightIntakeSensor");
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
        intakeMotor.setPower(-0.4);
    }

    public void intakePower(double power){
        intakeMotor.setPower(power);
    }

    public void intakePosition(double position){
        armServo.setPosition(position);
    }

    public void intakeDown(){
        armServo.setPosition(1.0);
        up = false;
    }

    public void intakeUp(){
        armServo.setPosition(0.16);
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
