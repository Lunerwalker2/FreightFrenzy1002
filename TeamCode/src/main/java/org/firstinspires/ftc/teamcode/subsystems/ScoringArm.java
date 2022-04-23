package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ScoringArm extends SubsystemBase {


    private final ServoImplEx servo;


    private final double loadPosition = 0.99;
    private final double scoringPosition = 0.43;

    public boolean loading = true;

    public ScoringArm(HardwareMap hardwareMap){
        servo = hardwareMap.get(ServoImplEx.class, "scoringArmServo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
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
