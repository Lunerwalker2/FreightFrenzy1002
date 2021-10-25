package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs
import kotlin.math.cos

@Config
class Arm(private val hardwareMap: HardwareMap) : SubsystemBase() {

    //Calling the constructor of the superclass already registers this subsystem

    private val armMotor by lazy {
        hardwareMap.get(DcMotorEx::class.java, "arm")
    }
    var armState = ArmState.STOPPED

    private var power = 0.0

    private var firstRun = true
    
    @JvmStatic
    var coefficients = PIDCoefficients(0.06, 0.0, 0.0)
    
    @JvmStatic
    var armGravityFeedforward: Double = 2.0

    //TODO: Test this
    private val armGravityController = PIDFController(coefficients,
            kF = { position, _ ->
                val angle = Range.scale(position, 0.0, armMaxAngleTicks.toDouble(), armMinAngle, armMaxAngle) //scale encoder ticks from 0 to top of the arm to degrees
                cos(angle) * armGravityFeedforward
            }
    )

    
    
    companion object {
        private const val armMinAngle = -40.0
        private const val armMaxAngle = 90.0

        private const val armMaxAngleTicks = 7000 //TODO: Find this

        private const val positionTolerance = 20

    }


    enum class ArmState {
        MOVING_MANUAL,
        MOVING_AUTO,
        HOLDING,
        STOPPED
    }

    //TODO: Find these
    enum class ArmPosition(val targetPosition: Int) {
        DOWN(0),
        BOTTOM_LEVEL(1000),
        MIDDLE_LEVEL(2000),
        TOP_LEVEL(3000),
        CAP_LEVEL(4000)
    }

    init{
        register()
    }


    override fun periodic() {
        if(firstRun){
            armMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            armMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        when (armState) {
            ArmState.STOPPED -> { //if we are stopped, set it to 0.
                armMotor.power = 0.0
            }
            ArmState.HOLDING -> { //If we are holding, keep the arm at the position it was stopped at
                armMotor.power = armGravityController.update(armMotor.currentPosition.toDouble()) //Hold the
            }
            ArmState.MOVING_AUTO -> {
                val currentPosition = armMotor.currentPosition //Store it so we cna avoid reading twice for bulk reading
                if (abs(currentPosition - armGravityController.targetPosition) <= positionTolerance) {
                    armGravityController.targetPosition = currentPosition.toDouble()
                    armState = ArmState.HOLDING //If we are at the target position, hold the arm
                } else {
                    armMotor.power = armGravityController.update(currentPosition.toDouble()) //else, move towards the position
                }
            }
            ArmState.MOVING_MANUAL -> { //If we are in manual movement, set the arm to that power
                armMotor.power = power
            }
        }

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


}
