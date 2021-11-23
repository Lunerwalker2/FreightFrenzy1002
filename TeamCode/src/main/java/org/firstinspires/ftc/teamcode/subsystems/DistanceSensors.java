package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MB1242;

public class DistanceSensors extends SubsystemBase {


    //forward sensor
    private final MB1242 forwardSensor;
    //The two TOF Distance sensors on the sides.
    private final Rev2mDistanceSensor rightSensor;
    private final Rev2mDistanceSensor leftSensor;

    //Timer object for the delay
    private final ElapsedTime delayTimer = new ElapsedTime();
    //Delay in ms between pings,
    private final int readingDelayMs = 80;
    //Whether we should be active
    private boolean takingRangeReading = false;
    //Last read range in cm
    private double forwardRangeReading = 0.0;
    private double leftRange = 0.0;
    private double rightRange = 0.0;

    private boolean firstRun = true;

    public DistanceSensors(HardwareMap hardwareMap){
        super();

        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(MB1242.class, "forwardSensor");
        rightSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class,"leftSensor");

    }

    @Override
    public void periodic(){
        //Only take readings if we are supposed to, it's an unnecessary hardware call otherwise
        if(takingRangeReading){
            //Check if its been longer than our delay to properly let the sensor perform
            if(hasDelayExpired() && !firstRun){
                forwardRangeReading = forwardSensor.readRange();
                //TODO: Read and store the left and right sensors IN CENTIMETERS (DistanceUnit.CM)
                rightRange = rightSensor.getDistance(DistanceUnit.CM);
                leftRange = leftSensor.getDistance(DistanceUnit.CM);
                forwardSensor.ping();
                delayTimer.reset();
                //If its the first run then run a range command to ensure the next reading has a value
            } else if(firstRun){
                forwardSensor.ping();
                delayTimer.reset();
                firstRun = false;
            }
        }
    }


    //Get the left sensor range in cm
    public double getLeftRange(DistanceUnit unit) {
        return unit.fromCm(leftRange);
    }

    public double getRightRange(DistanceUnit unit) {
        return unit.fromCm(rightRange);
    }

    public double getForwardRange(DistanceUnit unit) {
        return forwardRangeReading;
    }

    //Start taking range readings
    public void startReading() {
        takingRangeReading = true;
    }

    //Stop taking range readings
    public void stopReading() {
        takingRangeReading = false;
    }


    //Tells if the delay timer has expired or not
    public boolean hasDelayExpired() {
        return delayTimer.milliseconds() >= readingDelayMs;
    }


    //Tests the sensor by running a range command and seeing if there's an output, takes at least 100ms
    public boolean test() {
        forwardSensor.ping();
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < 100);
        return forwardSensor.readRange() != 20; //20cm is the minimum range, so we test with it
    }
}
