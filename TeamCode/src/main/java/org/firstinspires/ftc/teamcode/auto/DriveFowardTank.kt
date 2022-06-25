package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Autonomous
class DriveFowardTank(): LinearOpMode() {


    override fun runOpMode() {

        val lf = hardwareMap.get(DcMotor::class.java, "lf")
        val lb = hardwareMap.get(DcMotor::class.java, "lb")

        val rf = hardwareMap.get(DcMotor::class.java, "rf")

        val rb = hardwareMap.get(DcMotor::class.java, "rb")



        rf.direction = DcMotorSimple.Direction.REVERSE
        rb.direction = DcMotorSimple.Direction.REVERSE


        waitForStart()

        sleep(1000)
        lf.power = 1.0
        lb.power = 1.0
        rf.power = 1.0
        rb.power = 1.0
        sleep(1000)
        lf.power = 0.0
        lb.power = 0.0
        rf.power = 0.0
        rb.power = 0.0

    }
}