package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bucket extends SubsystemBase {

    private final Servo bucketServo;
    private final Telemetry telemetry;
    private boolean isDown = false;

    public Bucket(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public Bucket(HardwareMap hardwareMap, Telemetry telemetry){
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        dump();
        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        if(telemetry != null){
            telemetry.addData("Bucket Down", isDown);
        }
    }

    /**
     * Moves the bucket down to the depositing position.
     */
    public void load(){
        bucketServo.setPosition(1.0);
        isDown = true;
    }

    /**
     * Moves the bucket up to the holding (flat) position.
     */
    public void dump(){
        bucketServo.setPosition(0.0);
        isDown = false;
    }

    /**
     * Returns if the bucket is down (in the deposit position) or not.
     * @return Whether the bucket is down.
     */
    public boolean isDown(){
        return isDown;
    }


}
