package org.firstinspires.ftc.teamcode.testing.teleOp

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import kotlin.properties.Delegates

@TeleOp(name = "Momentum K-1.0")
class MomentumKT : LinearOpMode() {
    private var power = 1.0
    private var rightTrigger = 0f
    private var leftTrigger = 0f
    private var dPadRight = false
    private var dPadLeft = false
    private var dPadUp = false
    private var dPadDown = false
    private var rightBumper = false
    private var leftBumper = false

    private var slowLock = false
    private val duckMultiplier = 0.7
    private val scoreValue = 0.8
    private val startValue = 0.33
    override fun runOpMode() {
        val imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()

        val rightFront = hardwareMap.get(DcMotor::class.java, "rf")
        val leftFront = hardwareMap.get(DcMotor::class.java, "lf")
        val rightBack = hardwareMap.get(DcMotor::class.java, "rb")
        val leftBack = hardwareMap.get(DcMotor::class.java, "lb")
        val carouselMotor = hardwareMap.get(DcMotor::class.java, "carouselMotor")
        val frontIntake = hardwareMap.get(DcMotor::class.java, "frontIntake")
        val backIntake = hardwareMap.get(DcMotor::class.java, "backIntake")
        val liftMotor = hardwareMap.get(DcMotor::class.java, "liftMotor")
        val frontFlap = hardwareMap.get(Servo::class.java, "frontFlap")
        val backFlap = hardwareMap.get(Servo::class.java, "backFlap")
        val bucketServo = hardwareMap.get(Servo::class.java, "bucketServo")
        val scoringArmServo = hardwareMap.get(Servo::class.java, "scoringArmServo")

        var angle by Delegates.notNull<Double>()
        var leftBackPower by Delegates.notNull<Double>()
        var rightBackPower by Delegates.notNull<Double>()
        var leftFrontPower by Delegates.notNull<Double>()
        var rightFrontPower by Delegates.notNull<Double>()

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        rightFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        liftMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
            val vector = Vector2d(gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble()).rotated(angle)
            val (x, y) = vector.rotated(angle)
            dPadRight = gamepad1.dpad_right
            dPadLeft = gamepad1.dpad_left
            rightBumper = gamepad1.right_bumper
            leftBumper = gamepad1.left_bumper
            rightTrigger = gamepad1.right_trigger
            leftTrigger = gamepad1.left_trigger
            dPadUp = gamepad1.dpad_up
            dPadDown = gamepad1.dpad_down
            power = if (gamepad1.a && !slowLock) {
                0.5
            } else {
                1.0
            }
            slowLock = gamepad1.a
            leftFrontPower = power * (y + x + gamepad1.right_stick_x)
            leftBackPower = power * (y - x + gamepad1.right_stick_x)
            rightFrontPower = power * (y - x - gamepad1.right_stick_x)
            rightBackPower = power * (y + x - gamepad1.right_stick_x)
            leftFront.power = leftFrontPower
            rightFront.power = (rightFrontPower)
            leftBack.power = (leftBackPower)
            rightBack.power = (rightBackPower)
            if (dPadLeft || dPadRight) {
                var multiplier = 1
                if (dPadLeft) multiplier = -1
                carouselMotor.power = (multiplier * duckMultiplier)
            } else if (carouselMotor.power != 0.0) {
                carouselMotor.power = (0.0)
            }
            when {
                rightTrigger > 0 -> {
                    frontIntake.power = (-1.0)
                    setServoPosition(frontFlap, 1.0)
                }
                rightBumper -> {
                    frontIntake.power = (1.0)
                    setServoPosition(frontFlap, 1.0)
                }
                else -> {
                    frontIntake.power = (0.0)
                    backIntake.power = (0.0)
                    setServoPosition(frontFlap, 0.0)
                }
            }
            when {
                leftTrigger > 0 -> {
                    backIntake.power = (-1.0)
                    setServoPosition(backFlap, 1.0)
                }
                leftBumper -> {
                    backIntake.power = (1.0)
                    setServoPosition(backFlap, 1.0)
                }
                else -> {
                    frontIntake.power = (0.0)
                    backIntake.power = (0.0)
                    setServoPosition(backFlap, 0.0)
                }
            }
            when {
                dPadUp -> {
                    liftMotor.power = (.5)
                }
                dPadDown -> {
                    liftMotor.power = (-.5)
                }
                else -> {
                    liftMotor.power = (0.0)
                }
            }
            if (gamepad1.y) {
                scoringArmServo.position = (0.7)
            } else if (gamepad1.a) {
                scoringArmServo.position = (0.0)
            }
            if (gamepad1.x) {
                bucketServo.position = (startValue)
            } else if (gamepad1.b) {
                bucketServo.position = (scoreValue)
            }
        }
    }

    private fun setServoPosition(servo: Servo?, position: Double) {
        servo!!.position = position
    }
}