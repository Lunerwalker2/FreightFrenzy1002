package org.firstinspires.ftc.teamcode.teleOp

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow
import kotlin.properties.Delegates

@TeleOp(name = "Main TeleOp")
class MainTeleOp : LinearOpMode() {

    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightBack: DcMotorEx

    private lateinit var imu: BNO055IMU

    override fun runOpMode() {


        leftFront = hardwareMap.get(DcMotorEx::class.java, "lf")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "lb")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rf")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rb")



        rightFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        //Set the zero power behavior to brake
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE

        //Initialize IMU
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        //Send a line of text to the driver station screen
        telemetry.addLine("Ready For Start!")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {





            //Read the gamepad sticks
            var y = if (abs(gamepad1.left_stick_y) > 0.02) (-gamepad1.left_stick_y).toDouble() else 0.0
            var x = if (abs(gamepad1.left_stick_x) > 0.02) gamepad1.left_stick_x * 1.1 else 0.0
            var r = if (abs(gamepad1.right_stick_x) > 0.02) gamepad1.right_stick_x.toDouble() else 0.0

            //Cube the inputs for easier control
            y = cubeInput(y)
            x = cubeInput(x)
            r = cubeInput(r)

            //Read the imu angle
            val angle =
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
            //Rotate input vector by heading
            val vector = Vector2d(x, y).rotated(-angle)

            //Set the inputs to the rotated vector
            x = vector.x
            y = vector.y

            //Check for slow mode
            val power = if (gamepad1.left_bumper) 0.5 else 1.0

            //Normalize the powers if needed so that they are never >1.0
            val denominator: Double = max(abs(y) + abs(x) + abs(r), 1.0)

            //Find the correct motor powers
            val leftFrontPower = (power / denominator) * (y + x + r)
            val leftBackPower = (power / denominator) * (y - x + r)
            val rightFrontPower = (power / denominator) * (y - x - r)
            val rightBackPower = (power / denominator) * (y + x - r)

            //Set the motor powers
            leftFront.power = leftFrontPower
            rightFront.power = rightFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
        }

        leftFront.power = 0.0
        leftBack.power = 0.0
        rightFront.power = 0.0
        rightBack.power = 0.0
    }

    private fun cubeInput(input: Double): Double {
        val t = 0.2 * input.pow(3.0)
        val r = input * (1 - 0.2)
        return t + r
    }
}