package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import kotlin.math.abs

class Lift(private val hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) : SubsystemBase() {
    private var liftMotor: DcMotorEx

    enum class Positions(val targetPosition: Int) {
        TOP(1050),
        MIDDLE(500),
        BOTTOM(200),
        IN_ROBOT(0)
    }

    enum class State {
        MOVING,
        STOPPED
    }

    private var state = State.STOPPED

    init {
        liftMotor = hardwareMap.get(DcMotorEx::class.java, "liftMotor")

        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        liftMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun periodic() {
        when (state) {
            State.STOPPED -> {

            }
            State.MOVING -> {

            }
        }
    }

    fun setLiftPower(power: Double) = run { liftMotor.power = power }

    fun stopLift() {
        liftMotor.power = 0.0
        state = State.STOPPED
    }

    fun getLiftRawPosition(): Double {
        return liftMotor.currentPosition.toDouble()
    }


}