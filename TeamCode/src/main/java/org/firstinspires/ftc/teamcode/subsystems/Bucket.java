package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bucket extends SubsystemBase {

    private final Servo bucketServo;
    private final Servo sensorServo;
    private final RevColorSensorV3 colorSensor;
    private final Telemetry telemetry;
    private boolean isDown = false;

    public Bucket(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public Bucket(HardwareMap hardwareMap, Telemetry telemetry){
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        sensorServo = hardwareMap.get(Servo.class, "sensorServo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
        load();
        sensorUp();
        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        if(telemetry != null){
            telemetry.addData("Bucket Down", isDown);
        }
    }

    public boolean isFreightDetected(){
        return colorSensor.getRawLightDetected() > 200;
    }

    /**
     * Moves the bucket down to the depositing position.
     */
    public void load(){
        bucketServo.setPosition(0.0);
        isDown = true;
    }

    /**
     * Moves the bucket up to the holding (flat) position.
     */
    public void dump(){
        bucketServo.setPosition(1.0);
        isDown = false;
    }

    /**
     * Returns if the bucket is down (in the deposit position) or not.
     * @return Whether the bucket is down.
     */
    public boolean isDown(){
        return isDown;
    }

    public void sensorDown(){
        sensorServo.setPosition(1.0);
    }

    public void sensorUp(){
        sensorServo.setPosition(0.0);
    }


}
