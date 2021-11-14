package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.util.MB1242

class DistanceSensors(private val hardwareMap: HardwareMap, private val telemetry: Telemetry?): SubsystemBase() {

    private val forwardSensor: MB1242
    private val delayTimer = ElapsedTime()
    private val readingDelayMs: Int = 80
    private var takingRangeReading = false
    private var currentRangeReading: Int = 0
    private var firstRun = true

    init {
        forwardSensor = hardwareMap.get(MB1242::class.java, "forwardSensor")
    }

    override fun periodic() {
        //Only take readings if we are supposed to, it's an unnecessary hardware call otherwise
        if(takingRangeReading){
            //Check if its been longer than our delay to properly let the sensor perform
            if(hasDelayExpired() && !firstRun){
                currentRangeReading = forwardSensor.readRangeValueCm().toInt()
                forwardSensor.initiateRangeCommand()
                delayTimer.reset()
                firstRun = true
                //If its the first run then run a range command to ensure the next reading has a value
            } else if(firstRun){
                forwardSensor.initiateRangeCommand()
                delayTimer.reset()
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

    //Read the current range value in cm, is only updated every 80ms so might be a little inaccurate
    fun getCurrentRange(): Int {
        return currentRangeReading
    }


    //Tells if the delay timer has expired or not
    fun hasDelayExpired(): Boolean {
        return delayTimer.milliseconds() >= readingDelayMs
    }


    //Tests the sensor by running a range command and seeing if there's an output, takes at least 100ms
    fun test(): Boolean {
        forwardSensor.initiateRangeCommand()
        val timer = ElapsedTime()
        while(timer.milliseconds() < 100);
        return forwardSensor.readRangeValueCm().toInt() != 20
    }

}