package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RightIntake extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Servo armServo;

    public RightIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        armServo = hardwareMap.get(Servo.class, "rightIntakeArm");
    }

    @Override
    public void periodic(){

    }
}
