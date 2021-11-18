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
    //TODO: Add range variables for left and right
    private var forwardRangeReading: Int = 0
    private var leftRange = 0;
    private var rightRange = 0;

    //Whether this is the first run of the program
    private var firstRun = true

    init {
        //Get the sensors from the hardware map
        forwardSensor = hardwareMap.get(MB1242::class.java, "forwardSensor")
        rightSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "rightSensor")
        //TODO: Add other rev 2m sensor called "leftSensor" in the hardware map
        leftSensor = hardwareMap.get(Rev2mDistanceSensor::class.java,"leftSensor")
    }

    //Gets updated constantly throughout auto.
    override fun periodic() {
        //Only take readings if we are supposed to, it's an unnecessary hardware call otherwise
        if(takingRangeReading){
            //Check if its been longer than our delay to properly let the sensor perform
            if(hasDelayExpired() && !firstRun){
                forwardRangeReading = forwardSensor.readRange().toInt()
                //TODO: Read and store the left and right sensors IN CENTIMETERS (DistanceUnit.CM)
                rightRange = rightSensor.getDistance(DistanceUnit.CM).toInt()
                leftRange = leftSensor.getDistance(DistanceUnit.CM).toInt()
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

    /*
    TODO: Make a function that takes a boolean representing the side of the field (i.e. boolean redSide)
    public boolean redSide(boolean redSide) {
        if (redSide) {
            redSide = true;
        }
        else if (!redSide) {
            blueSide = true;
        }
        else {
            System.out.println("you screwed up lmao");
        }
    }
    TODO: and returns an array list with the forward reading in the first index and the side reading
    TODO: in the second. Look at the hasDelayExpired to see how to properly read the mb1242. The
    TODO: rev TOF sensor can just be directly read with no delay needed. DISTANCE IN CM
     */

    fun getRanges(redSide: Boolean): ArrayList<Double> {
        //Kotlin requires type consistency, so you might have to call .toDouble() on values which aren't
        //doubles, unlike Java which implicitly casts primitives.
        val list = ArrayList<Double>();

        list.add(forwardRangeReading.toDouble());
        if (redSide) {
            list.add(rightRange.toDouble());
        }
        else {
            list.add(leftRange.toDouble());
        }

        return list;

    }





    //TODO: Add get range methods for the left and right sensors
    fun getLeftRange(): Int {
        return leftRange
    }

    fun getRightRange(): Int {
        return rightRange
    }

    /**
     * Read the current range value in cm, is only updated every 80ms so might be a little inaccurate
     */
    fun getForwardRange(): Int {
        return forwardRangeReading
    }

    /**
     * Returns the distance in the specified value
     */
    fun getForwardRange(distanceUnit: DistanceUnit): Double{
        return distanceUnit.fromCm(forwardRangeReading.toDouble())
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