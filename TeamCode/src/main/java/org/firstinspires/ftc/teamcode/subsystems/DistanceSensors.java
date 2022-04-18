package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MB1242;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;

public class DistanceSensors extends SubsystemBase {

    //forward sensors
    private final MB1242 forwardSensor;
    private final MB1242 backwardSensor;
    //The TOF Distance sensor on the sides.
    private final AsyncRev2MSensor leftSensor;

    //Debugging rate timer
    private final ElapsedTime cycleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double cycleTime = 0.0;

    public DistanceSensors(HardwareMap hardwareMap) {

        //Get the sensors from the hardware map
        //TODO: YES I KNOW ITS REVERSED BUT I REFUSE TO SWITCH THE WIRING AROUND NOW
        forwardSensor = hardwareMap.get(MB1242.class, "forwardSensor");
        backwardSensor = hardwareMap.get(MB1242.class, "backwardSensor");
        leftSensor =
                new AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor.class, "leftSensor"));

        leftSensor.setMeasurementIntervalMs(60);

//        disableAll();
        cycleTimer.reset();

    }

    public void pingAll(){
        forwardSensor.ping();
        backwardSensor.ping();
    }

//    public void enableAll(){
//        leftSensor.enable();
//        forwardSensor.enable();
//        backwardSensor.enable();
//    }
//
//    public void disableAll(){
//        leftSensor.disable();
//        forwardSensor.disable();
//        backwardSensor.disable();
//    }

    @Override
    public void periodic() {
        cycleTime = cycleTimer.milliseconds();
        cycleTimer.reset();
    }


    //Get the left sensor range in cm
    public double getLeftRange(DistanceUnit unit) {
        return unit.fromCm(leftSensor.getDistance(DistanceUnit.CM));
    }

    public double getForwardRange(DistanceUnit unit) {
        return unit.fromCm(forwardSensor.getDistance(DistanceUnit.CM));
    }

    public double getBackwardRange(DistanceUnit unit) {
        return unit.fromCm(backwardSensor.getDistance(DistanceUnit.CM));
    }

    /**
     * Returns the cycle time in milliseconds of the distance sensors.
     */
    public double getCycleTime() {
        return cycleTime;
    }

    //Tests the sensor by running a range command and seeing if there's an output, takes at least 100ms
    public boolean test() {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 80) ;
        return forwardSensor.getDistance(DistanceUnit.CM) > 20 || //20cm is the minimum range, so we test with it
                leftSensor.getDistance(DistanceUnit.CM) > 3 ||
                backwardSensor.getDistance(DistanceUnit.CM) > 20;
    }
}
