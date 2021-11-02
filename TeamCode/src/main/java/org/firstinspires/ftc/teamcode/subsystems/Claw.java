package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {


    private final Servo clawServo;

    //Since the hardware map should be set
    public Claw(HardwareMap hardwareMap){
        super(); //registers this subsystem
        clawServo = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void periodic(){

    }

    public void openClaw(){
        clawServo.setPosition(0.2);
    }

    public void closeClaw(){
        clawServo.setPosition(0.6);
    }


}
