package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp
class ArmPositionOutput: LinearOpMode() {


    override fun runOpMode() {
        val arm = hardwareMap.get(DcMotorEx::class.java, "arm")

        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        while (opModeIsActive()){
            telemetry.addData("Arm current position", arm.currentPosition)
            telemetry.update()
        }
    }
}