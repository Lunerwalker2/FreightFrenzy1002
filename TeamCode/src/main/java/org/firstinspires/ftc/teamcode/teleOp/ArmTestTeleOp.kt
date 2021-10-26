package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor


@TeleOp
class ArmTestTeleOp : LinearOpMode() {


    override fun runOpMode() {
        val arm = hardwareMap.get(DcMotor::class.java, "arm")


        waitForStart()

        while(opModeIsActive()){
            telemetry.addData("Arm encoder position: ", arm.currentPosition)
            telemetry.update()
        }
    }
}