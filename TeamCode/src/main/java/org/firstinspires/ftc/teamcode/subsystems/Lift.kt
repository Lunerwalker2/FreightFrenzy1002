package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import kotlin.math.abs

class Lift(private val hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) : SubsystemBase() {
    private var deposit: Servo
    private var liftMotor: DcMotorEx

    enum class Positions(val targetPosition: Int) {
        TOP(650),
        MIDDLE(400),
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

        deposit = hardwareMap.get(Servo::class.java, "deposit")
    }

    override fun periodic() {
        when (state) {
            State.STOPPED -> {

            }
            State.MOVING -> {

            }
        }
    }

    fun stopLift(){
        liftMotor.power = 0.0
        state = State.STOPPED
    }



}