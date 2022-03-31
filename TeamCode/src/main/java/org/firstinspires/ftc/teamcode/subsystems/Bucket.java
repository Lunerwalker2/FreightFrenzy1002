package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket extends SubsystemBase {




    private final Servo servo;

    private final double openPosition = 0.6;
    private final double closePosition = 1.0;

    public Bucket(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "bucketServo");
    }


    @Override
    public void periodic(){
        //happens every loop
    }

    public void open(){
        servo.setPosition(openPosition);
    }

    public void close(){
        servo.setPosition(closePosition);
    }
}
