package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.util.Extensions
import kotlin.math.abs

@TeleOp
class TankDriveTeleOp(): LinearOpMode() {


    private lateinit var leftFront: DcMotor
    private lateinit var leftBack: DcMotor
    private lateinit var rightFront: DcMotor
    private lateinit var rightBack: DcMotor


    @Throws(InterruptedException::class)
    override fun runOpMode() {

        leftFront = hardwareMap.get(DcMotor::class.java, "lf")
        leftBack = hardwareMap.get(DcMotor::class.java, "lb")
        rightFront = hardwareMap.get(DcMotor::class.java, "rf")
        rightBack = hardwareMap.get(DcMotor::class.java, "rb")

        rightFront.direction = DcMotorSimple.Direction.REVERSE
        rightBack   .direction = DcMotorSimple.Direction.REVERSE

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        leftFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


        waitForStart()

        while (opModeIsActive()){

            val y = Extensions.cubeInput(-gamepad1.left_stick_y.toDouble(), 0.2)
            val r = Extensions.cubeInput(-gamepad1.right_stick_x.toDouble(), 0.2)

            var leftSpeed: Double
            var rightSpeed: Double


            if(!gamepad1.left_bumper){
                //arcade drive
                leftSpeed = y + r
                rightSpeed = y - r
            } else {
                leftSpeed = y + abs(y) * r
                rightSpeed = y - abs(y) * r
            }

            val maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed))
            if (maxMagnitude > 1.0) {
                leftSpeed /= maxMagnitude
                rightSpeed /= maxMagnitude
            }

            leftFront.power = leftSpeed
            leftBack.power = leftSpeed
            rightFront.power = rightSpeed
            rightBack.power = rightSpeed

        }

    }
}