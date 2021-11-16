package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.MB1242


class DistanceSensors(private val hardwareMap: HardwareMap, private val telemetry: Telemetry?): SubsystemBase() {


    //Distance sensors

    //The ultrasonic sensor facing forward
    private val forwardSensor: MB1242

    //The two TOF Distance sensors on the sides.
    private val rightSensor: Rev2mDistanceSensor
    private val leftSensor: Rev2mDistanceSensor

    //Timer object for the delay
    private val delayTimer = ElapsedTime()

    //Delay in ms between pings, recommended 80-100ms for the ultrasonic sensor
    private val readingDelayMs: Int = 80

    //Whether we should be actively reading the range or not
    private var takingRangeReading = false

    //Last read range in cm
    private var currentRangeReading: Int = 0

    //Whether this is the first run of the program
    private var firstRun = true

    init {
        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(MB1242::class.java, "forwardSensor")
    }

    override fun periodic() {
        //Only take readings if we are supposed to, it's an unnecessary hardware call otherwise
        if(takingRangeReading){
            //Check if its been longer than our delay to properly let the sensor perform
            if(hasDelayExpired() && !firstRun){
                currentRangeReading = forwardSensor.readRange().toInt()
                forwardSensor.ping()
                delayTimer.reset()
                //If its the first run then run a range command to ensure the next reading has a value
            } else if(firstRun){
                forwardSensor.ping()
                delayTimer.reset()
                firstRun = false
            }
        }
    }


    //Start taking range readings
    fun startReading(){
        takingRangeReading = true
    }

    //Stop taking range readings
    fun stopReading(){
        takingRangeReading = false
    }

    /**
     * Read the current range value in cm, is only updated every 80ms so might be a little inaccurate
     */
    fun getCurrentRange(): Int {
        return currentRangeReading
    }

    /**
     * Returns the distance in the specified value
     */
    fun getCurrentRange(distanceUnit: DistanceUnit): Double{
        return distanceUnit.fromCm(currentRangeReading.toDouble())
    }


    //Tells if the delay timer has expired or not
    fun hasDelayExpired(): Boolean {
        return delayTimer.milliseconds() >= readingDelayMs
    }


    //Tests the sensor by running a range command and seeing if there's an output, takes at least 100ms
    fun test(): Boolean {
        forwardSensor.ping()
        val timer = ElapsedTime()
        while(timer.milliseconds() < 100);
        return forwardSensor.readRange().toInt() != 20
    }

}