package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs
import kotlin.math.cos

@Config
class Arm(private val hardwareMap: HardwareMap, private val telemetry: Telemetry?) : SubsystemBase() {

    constructor(hardwareMap: HardwareMap) : this(hardwareMap, null)

    //Calling the constructor of the superclass already registers this subsystem

    private val armMotor by lazy { hardwareMap.get(DcMotorEx::class.java, "arm") }


    var armState = ArmState.STOPPED

    private var power = 0.0
    private var firstRun = true

    //The annotations mean it can be seen by ftc dashboard (in kotlin)
    //Equivalent of doing public static in java.
    @JvmField
    var coefficients = PIDCoefficients(0.002, 0.0, 0.0)

    @JvmField
    var armGravityFeedforward: Double = 0.4 //TODO: Find this

    //TODO: Test this
    /*
    See the book Controls Engineering in FRC for an explanation of this and the equation
    for a theoretical gravity feedforward constant.

    Fg = m*g*(L/2)*cos(angle of arm)
     */


    private val armGravityController = PIDFController(coefficients,
            kF = { position, _ ->
                findGravityFF(position)
            }
    )

    companion object {

        //The tolerance of our controller in ticks
        private const val positionTolerance = 10

        //Encoder ticks per revolution of our motor
        private const val TICKS_PER_REV = 2100.0

        //Distance in the encoder ticks from the bottom limit of the arms rotation to horizontal
        private const val ARM_TO_HORIZONTAL_TICKS_OFFSET = 324.0

        fun getTicksPerRev(): Double {
            return TICKS_PER_REV
        }
    }


    //Current movement state of arm
    enum class ArmState {
        MOVING_MANUAL,
        MOVING_AUTO,
        HOLDING,
        STOPPED
    }

    //TODO: Find these
    //Encoder positions of the arm motor for different levels
    enum class ArmPosition(val targetPosition: Int) {
        DOWN(0),
        SCORING_LEVEL(1077),
//        BOTTOM_LEVEL(41),
//        MIDDLE_LEVEL(73),
//        TOP_LEVEL(364)
    }

    init {
        //We can raise this but to be safe we are leaving it here for now
        armGravityController.setOutputBounds(-0.4, 0.6)
        register()
    }


    override fun periodic() {
        //On the first run set the zero power behavior and run mode
        if (firstRun) {
            armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            armMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            firstRun = false
        }

        val currentPosition = getArmPosition()

        telemetry?.addData("Arm current state", armState)

        when (armState) {
            ArmState.STOPPED -> { //if we are stopped, set it to 0.
                armMotor.power = 0.0
            }
            ArmState.HOLDING -> { //If we are holding, keep the arm at the position it was stopped at

                //If the current position is within our tolerance then just apply the gravity ff
                if (abs(armGravityController.targetPosition - currentPosition) <= positionTolerance) {
                    armMotor.power = findGravityFF(currentPosition)
                } else {
                    //Otherwise try to get back there
                    armMotor.power = armGravityController.update(currentPosition)
                }
            }
            //If we are moving to a position, update the motor powers with the controller
            ArmState.MOVING_AUTO -> {

                //Check if the current position is within our tolerance range
                if (abs(armGravityController.targetPosition - currentPosition) <= positionTolerance) {

                    //If it is, then make the new target the currentPosition
                    if (currentPosition > 15) {
                        hold()
                    } else {
                        stop()
                    }
                } else {
                    armMotor.power = armGravityController.update(currentPosition) //else, move towards the position
                }
            }
            ArmState.MOVING_MANUAL -> { //If we are in manual movement, set the arm to that power
                armMotor.power = power
            }
        }
        compileTelemetry(currentPosition.toInt())

    }


    //Sets the arm to a manual power
    fun armPower(power: Double) {
        this.power = power
        armState = ArmState.MOVING_MANUAL
    }

    //Moves the arm to the specified position
    fun setArm(position: ArmPosition) {
        armGravityController.targetPosition = position.targetPosition.toDouble()
        armState = ArmState.MOVING_AUTO
    }

    //Holds the arm at the current angle
    fun hold() {
        armGravityController.targetPosition = armMotor.currentPosition.toDouble()
        armState = ArmState.HOLDING
    }


    //Sets a 0 power to the arm
    fun stop() {
        armState = ArmState.STOPPED
    }

    fun getArmPosition(): Double {
        return armMotor.currentPosition.toDouble()
    }


    private fun findGravityFF(position: Double): Double {
        //Find the angle of the arm in degrees

        /*
        The arm starts at lower than 0 degrees so we do have to subtract the difference
        between that and the actual 0 position of the arm in ticks. Otherwise the controller
        would think that the arm started perfectly horizontal.
         */
        return cos(findArmAngle(position)) * armGravityFeedforward
    }

    private fun findArmAngle(position: Double): Double {
        return (position - ARM_TO_HORIZONTAL_TICKS_OFFSET) / TICKS_PER_REV * 360.0
    }

    private fun compileTelemetry(currentPosition: Int) {
        telemetry?.addData("Arm Target Position/Angle", "%d / %.2f", currentPosition, findArmAngle(currentPosition.toDouble()))
        telemetry?.addData("Arm Current Position/Angle", "%d / %.2f", currentPosition, findArmAngle(currentPosition.toDouble()))
        telemetry?.addData("Arm Current Power", "%.4f", armMotor.power)
        telemetry?.addData("Arm Gravity FF Correction", "%.4f", findGravityFF(currentPosition.toDouble()))
        //Represent the amperage draw out of the amperage alert with ascii art for fun
        telemetry?.addData("Arm Motor Current Draw", Func {
            val amperage = armMotor.getCurrent(CurrentUnit.AMPS)
            val numWhiteSquares = Range.scale(
                    amperage,
                    0.0,
                    armMotor.getCurrentAlert(CurrentUnit.AMPS),
                    0.0,
                    10.0
            ).toInt()
            val builder = StringBuilder()
            builder.append("[")
            for (i in 0..10) {
                if (i < numWhiteSquares) builder.append("\u2b1b")
                else builder.append("\u2b1c")
            }
            builder.append("] (%.4f A)".format(amperage))
            builder.toString()
        }.value()
        )
        telemetry?.addData("Arm Motor Over-current", armMotor.isOverCurrent)

    }

}
