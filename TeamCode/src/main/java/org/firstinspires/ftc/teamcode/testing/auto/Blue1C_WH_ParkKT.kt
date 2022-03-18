package org.firstinspires.ftc.teamcode.testing.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.Throws
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.PI
import kotlin.math.abs

@Autonomous(name = "Blue Warehouse 1-Cycle KOTLIN")
class Blue1C_WH_ParkKT : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val rightFront = hardwareMap.get(DcMotor::class.java, "rf")
        val leftFront = hardwareMap.get(DcMotor::class.java, "lf")
        val rightBack = hardwareMap.get(DcMotor::class.java, "rb")
        val leftBack = hardwareMap.get(DcMotor::class.java, "lb")
        val wheels = arrayOf(rightFront, leftFront, rightBack, leftBack)
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        for (motor in wheels) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
        waitForStart()
        encoderDrive(0.6, 20.0, 48.0, leftFront, rightFront, leftBack, rightBack)
        sleep(1000)
    }

    private fun encoderDrive(speed: Double, leftInches: Double, rightInches: Double,
                             leftFront: DcMotor, rightFront: DcMotor, leftBack: DcMotor, rightBack: DcMotor) {
        val lf: Int
        val rf: Int
        val rb: Int
        val lb: Int
        if (opModeIsActive()) {
            lf = leftFront.currentPosition + (leftInches * COUNTS_PER_INCH).toInt()
            rf = rightFront.currentPosition + (rightInches * COUNTS_PER_INCH).toInt()
            lb = leftBack.currentPosition + (leftInches * COUNTS_PER_INCH).toInt()
            rb = rightBack.currentPosition + (rightInches * COUNTS_PER_INCH).toInt()
            leftFront.targetPosition = lf
            rightFront.targetPosition = rf
            leftBack.targetPosition = lb
            rightBack.targetPosition = rb
            leftFront.mode = DcMotor.RunMode.RUN_TO_POSITION
            rightFront.mode = DcMotor.RunMode.RUN_TO_POSITION
            leftFront.power = abs(speed)
            rightFront.power = abs(speed)
            leftFront.power = 0.0
            rightFront.power = 0.0
            leftFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
            rightFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    companion object {
        private const val COUNTS_PER_MOTOR_REV = 537.7
        private const val GEAR_REDUCTION = 15.36
        private const val DIAMETER_IN = 3.77953
        const val COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION / (DIAMETER_IN * PI)
    }
}