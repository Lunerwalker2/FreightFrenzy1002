package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
class CarouselTestTeleOp: LinearOpMode() {


    override fun runOpMode() {

        val carousel = hardwareMap.get(DcMotor::class.java, "carousel")



        waitForStart()

        while (opModeIsActive()) {
            if (gamepad2.left_trigger > 0.2) carousel.power = 0.5
            else if (gamepad2.right_trigger > 0.2) carousel.power = -0.5
            else carousel.power = 0.0
        }
    }

}