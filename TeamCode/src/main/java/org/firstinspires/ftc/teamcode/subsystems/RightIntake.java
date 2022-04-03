package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;

public class RightIntake extends SubsystemBase implements IntakeSide {

    private final DcMotorEx intakeMotor;
    private final Servo armServo;

    public boolean up = true;

    public RightIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armServo = hardwareMap.get(Servo.class, "rightIntakeArm");
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

    public void intakeDown(){
        armServo.setPosition(0.8);
        up = false;
    }

    public void intakeUp(){
        armServo.setPosition(0.3);
        up = true;
    }

    public int currentPosition(){
        return intakeMotor.getCurrentPosition();
    }
}
