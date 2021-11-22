package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.MB1242

/*
When we do SubsystemBase() in kotlin here, it calls the super() constructor.
 */
class DistanceSensors(private val hardwareMap: HardwareMap, private val telemetry: Telemetry? = null): SubsystemBase() {


    //Distance sensors

    //The ultrasonic sensor facing forward
    private val forwardSensor: MB1242

    //The two TOF Distance sensors on the sides.
    private val rightSensor: Rev2mDistanceSensor
    //TODO: Uncomment this (cursor on line and then press Ctrl + /)
    private val leftSensor: Rev2mDistanceSensor


    //Timer object for the delay
    private val delayTimer = ElapsedTime()

    //Delay in ms between pings, recommended 80-100ms for the ultrasonic sensor
    private val readingDelayMs: Int = 80

    //Whether we should be actively reading the range or not
    private var takingRangeReading = false

    //Last read range in cm
    private var forwardRangeReading = 0.0
    private var leftRange = 0.0
    private var rightRange = 0.0

    //Whether this is the first run of the program
    private var firstRun = true

    init {
        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(MB1242::class.java, "forwardSensor")
        rightSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "rightSensor")
        leftSensor = hardwareMap.get(Rev2mDistanceSensor::class.java,"leftSensor")
    }

    //Gets updated constantly throughout auto.
    override fun periodic() {
        //Only take readings if we are supposed to, it's an unnecessary hardware call otherwise
        if(takingRangeReading){
            //Check if its been longer than our delay to properly let the sensor perform
            if(hasDelayExpired() && !firstRun){
                forwardRangeReading = forwardSensor.readRange().toDouble()
                //TODO: Read and store the left and right sensors IN CENTIMETERS (DistanceUnit.CM)
                rightRange = rightSensor.getDistance(DistanceUnit.CM)
                leftRange = leftSensor.getDistance(DistanceUnit.CM)
                forwardSensor.ping()
                delayTimer.reset()
                //If its the first run then run a range command to ensure the next reading has a value
            } else if(firstRun){
                forwardSensor.ping()
                delayTimer.reset()
                firstRun = false
            }
        }

        //TODO: If   you want, add telemetry calls here with the following
        telemetry?.addData("foo", 3);
        //Telemetry is updated later on probably
    }



    fun getRanges(redSide: Boolean): ArrayList<Double> {
        val list = ArrayList<Double>();

        list.add(forwardRangeReading);
        if (redSide) {
            list.add(rightRange);
        }
        else {
            list.add(leftRange);
        }

        return list;
    }





    //TODO: Add get range methods for the left and right sensors
    fun getLeftRange(): Double {
        return leftRange
    }

    fun getRightRange(): Double {
        return rightRange
    }

    /**
     * Read the current range value in cm, is only updated every 80ms so might be a little inaccurate
     */
    fun getForwardRange(): Double {
        return forwardRangeReading
    }

    /**
     * Returns the distance in the specified value
     */
    fun getForwardRange(distanceUnit: DistanceUnit): Double{
        return distanceUnit.fromCm(forwardRangeReading)
    }


    //Start taking range readings
    fun startReading(){
        takingRangeReading = true
    }

    //Stop taking range readings
    fun stopReading(){
        takingRangeReading = false
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
        return forwardSensor.readRange().toInt() != 20 //20cm is the minimum range, so we test with it
    }

}