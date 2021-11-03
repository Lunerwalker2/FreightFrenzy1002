package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Intake(private val hardwareMap: HardwareMap) : SubsystemBase() {

    //Epic minimal syntax kotlin superclass constructor calling time yay suck it java!

    //The by lazy thing means that it will be set the first time it is accessed
    private val intakeMotor by lazy { hardwareMap.get(DcMotor::class.java, "intake") }
    private val flapServo by lazy { hardwareMap.get(Servo::class.java, "flap") }

    private var power = 0.0

    private var firstRun = true


    override fun periodic(){
        if(firstRun){

            flapServo.position = 0.0

            firstRun = false
        }

        intakeMotor.power = power

    }


    fun closeFlap(){
        flapServo.position = 0.0
    }

    fun openFlap(){
        flapServo.position = 0.5
    }

    fun intake(){
        power = 1.0
    }

    fun outtake(){
        power = -1.0
    }

    fun stop(){
        power = 0.0
    }
}