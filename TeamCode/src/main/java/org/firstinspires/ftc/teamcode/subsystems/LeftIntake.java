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
    }

    @Override
    public void periodic(){

    }
}
