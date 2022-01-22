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
    private final MB1242 backwardSensor;
    //The two TOF Distance sensors on the sides.
    private final Rev2mDistanceSensor rightSensor;
    private final Rev2mDistanceSensor leftSensor;

    //Timer object for the delay
    private final ElapsedTime delayTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //Debugging rate timer
    private final ElapsedTime cycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double cycleTime = 0.0;
    //Delay in ms between pings
    private final int readingDelayMs = 60;
    //Whether we should be active
    private boolean takingRangeReading = false;
    //Last read range in cm
    private double forwardRange = 0.0;
    private double backwardRange = 0.0;
    private double leftRange = 0.0;
    private double rightRange = 0.0;

    public DistanceSensors(HardwareMap hardwareMap) {

        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(MB1242.class, "forwardSensor");
        backwardSensor = hardwareMap.get(MB1242.class, "backwardSensor");
        rightSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor");
        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftSensor");

        //Set initial values, usage must be careful to account for the first read as it might be incorrect
        forwardRange = forwardSensor.getDistance(DistanceUnit.CM);
        backwardRange = backwardSensor.getDistance(DistanceUnit.CM);
        leftRange = leftSensor.getDistance(DistanceUnit.CM);
        rightRange = rightSensor.getDistance(DistanceUnit.CM);

        cycleTimer.reset();
        delayTimer.reset();

    }


    @Override
    public void periodic() {
        //Only take readings if we are supposed to, it's an unnecessary hardware call otherwise
        if (takingRangeReading) {
            //Check if its been longer than our delay to properly let the sensor perform
            if (hasDelayExpired()) {
                backwardRange = backwardSensor.getDistance(DistanceUnit.CM);
                forwardRange = forwardSensor.getDistance(DistanceUnit.CM);
                backwardSensor.ping();
                forwardSensor.ping();
                rightRange = rightSensor.getDistance(DistanceUnit.CM);
                leftRange = leftSensor.getDistance(DistanceUnit.CM);
                delayTimer.reset();
            }
        }
        cycleTime = cycleTimer.milliseconds();
        cycleTimer.reset();
    }


    //Get the left sensor range in cm
    public double getLeftRange(DistanceUnit unit) {
        return unit.fromCm(leftRange);
    }

    public double getRightRange(DistanceUnit unit) {
        return unit.fromCm(rightRange);
    }

    public double getForwardRange(DistanceUnit unit) {
        return unit.fromCm(forwardRange);
    }

    public double getBackwardRange(DistanceUnit unit) {
        return unit.fromCm(backwardRange);
    }


    //Start taking range readings
    public void startReading() {
        takingRangeReading = true;
    }

    //Stop taking range readings
    public void stopReading() {
        takingRangeReading = false;
    }

    public boolean isTakingRangeReading() {
        return takingRangeReading;
    }


    //Tells if the delay timer has expired or not
    public boolean hasDelayExpired() {
        return delayTimer.milliseconds() >= readingDelayMs;
    }

    /**
     * Returns the cycle time in milliseconds of the distance sensors.
     */
    public double getCycleTime() {
        return cycleTime;
    }

    //Tests the sensor by running a range command and seeing if there's an output, takes at least 100ms
    public boolean test() {
        forwardSensor.ping();
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 80) ;
        return forwardSensor.getDistance(DistanceUnit.CM) > 20 || //20cm is the minimum range, so we test with it
                leftSensor.getDistance(DistanceUnit.CM) > 3 ||
                rightSensor.getDistance(DistanceUnit.CM) > 3 ||
                backwardSensor.getDistance(DistanceUnit.CM) > 20;
    }
}
