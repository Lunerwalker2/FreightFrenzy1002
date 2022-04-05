package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringArm extends SubsystemBase {


    private final Servo servo;


    private final double loadPosition = 0.16;
    private final double scoringPosition = 0.75;

    public boolean loading = true;

    public ScoringArm(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "scoringArmServo");
        loadingPosition();
    }

    @Override
    public void periodic(){
        //happens every loop
    }


    public void loadingPosition(){
        servo.setPosition(loadPosition);
        loading = true;
    }

    public void scoringPosition(){
        servo.setPosition(scoringPosition);
        loading = false;
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

}
