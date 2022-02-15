package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;

public class Bucket extends SubsystemBase {

    private final Servo bucketServo;
    private final AsyncRev2MSensor intakeSensor;
    private final Telemetry telemetry;
    private boolean isDown = false;

    public final double loadPosition = 0.33;
    public final double dumbPosition = 0.8;

    public Bucket(HardwareMap hardwareMap){
        this(hardwareMap, null);
    }

    public Bucket(HardwareMap hardwareMap, Telemetry telemetry){
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeSensor =
                new AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor.class, "intakeSensor"));
        load();
        intakeSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_SPEED);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        if(telemetry != null){
            telemetry.addData("Bucket Down", isDown);
        }
    }

    //TODO: Find correct value
    public boolean freightDetected(){
        return intakeSensor.getDistance(DistanceUnit.INCH) < 4;
    }

    /**
     * Moves the bucket down to the depositing position.
     */
    public void load(){
        bucketServo.setPosition(loadPosition);
        isDown = true;
    }

    /**
     * Moves the bucket up to the holding (flat) position.
     */
    public void dump(){
        bucketServo.setPosition(dumbPosition);
        isDown = false;
    }

    public void setPosition(double position){
        bucketServo.setPosition(position);
    }


    /**
     * Returns if the bucket is down (in the deposit position) or not.
     * @return Whether the bucket is down.
     */
    public boolean isDown(){
        return isDown;
    }


}
