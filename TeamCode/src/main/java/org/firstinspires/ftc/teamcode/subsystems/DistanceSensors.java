package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.testing.AsyncRev2MSensor;
import org.firstinspires.ftc.teamcode.util.MB1242;

public class DistanceSensors extends SubsystemBase {

    //forward sensor
    private final MB1242 forwardSensor;
    private final MB1242 backwardSensor;
    //The two TOF Distance sensors on the sides.
    private final AsyncRev2MSensor rightSensor;
    private final AsyncRev2MSensor leftSensor;

    //Debugging rate timer
    private final ElapsedTime cycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double cycleTime = 0.0;

    public DistanceSensors(HardwareMap hardwareMap) {

        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(MB1242.class, "forwardSensor");
        backwardSensor = hardwareMap.get(MB1242.class, "backwardSensor");
        rightSensor =
                new AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor"));
        leftSensor =
                new AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor.class, "leftSensor"));

        rightSensor.setMeasurementIntervalMs(60);
        cycleTimer.reset();

    }


    @Override
    public void periodic() {
        cycleTime = cycleTimer.milliseconds();
        cycleTimer.reset();
    }


    public void pingAll(){
        forwardSensor.ping();
        backwardSensor.ping();
    }

    //Get the left sensor range in cm
    public double getLeftRange(DistanceUnit unit) {
        return unit.fromCm(leftSensor.getDistance(DistanceUnit.CM));
    }

    public double getRightRange(DistanceUnit unit) {
        return unit.fromCm(leftSensor.getDistance(DistanceUnit.CM));
    }

    public double getForwardRange(DistanceUnit unit) {
        return unit.fromCm(leftSensor.getDistance(DistanceUnit.CM));
    }

    public double getBackwardRange(DistanceUnit unit) {
        return unit.fromCm(leftSensor.getDistance(DistanceUnit.CM));
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
