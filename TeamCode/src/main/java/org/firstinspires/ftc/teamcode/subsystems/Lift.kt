package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import org.firstinspires.ftc.robotcore.external.Telemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import kotlin.math.abs

class Lift(private val hardwareMap: HardwareMap, private val telemetry: Telemetry? = null) : SubsystemBase() {
    private val liftMotor: DcMotorEx

    enum class Positions(val targetPosition: Int) {
        TOP(651),    // 651
        MIDDLE(456), // 456
        BOTTOM(304), // 304
        IN_ROBOT(2)
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

        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun periodic() {
        telemetry?.addData("Lift Status", state)
        when (state) {
            State.STOPPED -> {

            }
            State.MOVING -> {

            }
        }
        telemetry?.addData("Lift Position", liftMotor.currentPosition)
        val t = TelemetryPacket()
        t.put("Lift Current Position", getLiftRawPosition())
        FtcDashboard.getInstance().sendTelemetryPacket(t)
    }

    fun setLiftPower(power: Double) = run { liftMotor.power = power }

    fun atUpperLimit(): Boolean {
        return getLiftRawPosition() > 1000
    }

    fun atLowerLimit(): Boolean {
        return getLiftRawPosition() < 0
    }

    fun stopLift() {
        liftMotor.power = 0.0
        state = State.STOPPED
    }

    fun getLiftRawPosition(): Double {
        return liftMotor.currentPosition.toDouble()
    }

    /**
     * Resets the lift motor encoder to the current position
     */
    fun resetZeroPosition(){
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


}