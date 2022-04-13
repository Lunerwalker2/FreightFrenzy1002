package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CappingMech extends SubsystemBase {


    private Servo servo;
    private CRServo crServo;

    private double position = 1.0;

    public CappingMech(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "cappingServo");
        crServo = hardwareMap.get(CRServo.class, "cappingCRServo");
        servo.setPosition(position);
    }

    public void incrementPosition(){
        double newPos = position + 0.02;
        if(newPos > 0.0){
            servo.setPosition(newPos);
            position = newPos;
        }
    }

    public void decrementPosition(){
        double newPos = position - 0.02;
        if(newPos > 1.0){
            servo.setPosition(newPos);
            position = newPos;
        }
    }

    public void extend(){
        crServo.setPower(1.0);
    }

    public void retract(){
        crServo.setPower(-1.0);
    }

    public void stop(){
        crServo.setPower(0);
    }
}
