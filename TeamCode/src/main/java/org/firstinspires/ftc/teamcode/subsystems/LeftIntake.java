package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LeftIntake extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Servo armServo;

    public LeftIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        armServo = hardwareMap.get(Servo.class, "leftIntakeArm");
        intakeUp();
    }

    @Override
    public void periodic(){

    }

    public void intake(){
        intakeMotor.setPower(0.7);
    }

    public void stop(){
        intakeMotor.setPower(0);
    }

    public void intakeDown(){
        armServo.setPosition(0);
    }

    public void intakeUp(){
        armServo.setPosition(1);
    }
}
