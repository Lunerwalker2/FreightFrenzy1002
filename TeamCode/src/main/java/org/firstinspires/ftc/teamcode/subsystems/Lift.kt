package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.abs

class Lift(private val hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) : SubsystemBase() {
    private var deposit: Servo
    private var liftMotor: DcMotorEx
    private var liftController: PIDFController

    enum class Positions(val targetPosition: Int) {
        TOP(100),
        MIDDLE(50),
        BOTTOM(10),
        IN_ROBOT(0)
    }

    enum class State {
        MOVING_AUTO,
        STOPPED
    }

    private var position = Positions.IN_ROBOT
    private var state = State.STOPPED

    companion object {
        var liftCoefficients = PIDCoefficients(0.1)
    }

    init {
        liftMotor = hardwareMap.get(DcMotorEx::class.java, "liftMotor")
        deposit = hardwareMap.get(Servo::class.java, "deposit")
        liftController = PIDFController(liftCoefficients, kStatic = 0.1)
        liftController.setOutputBounds(-1.0, 1.0)
    }

    override fun periodic() {

        when (state) {
            State.STOPPED -> {

            }
            State.MOVING_AUTO -> {
                if (position != Positions.IN_ROBOT && liftMotor.currentPosition >= 10) {
                    if (abs(position.targetPosition - liftMotor.currentPosition) > 5) {
                        liftMotor.power = liftController.update(liftMotor.currentPosition.toDouble())
                    }
                } else {
                    state = State.STOPPED
                    liftMotor.power = 0.0
                    position = Positions.IN_ROBOT
                }
            }
        }
    }


    fun setLiftPosition(position: Positions) {
        liftController.targetPosition = position.targetPosition.toDouble()
        state = State.MOVING_AUTO
    }

    fun stopLift(){
        liftMotor.power = 0.0
        state = State.STOPPED
    }

}