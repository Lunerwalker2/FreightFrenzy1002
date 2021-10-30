package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class IntakeTestTeleOp : LinearOpMode(){


    override fun runOpMode() {


        val claw = hardwareMap.get(Servo::class.java, "claw")

        waitForStart()

        if (isStopRequested) return

        var wasPressed = false
        var clawOpen = false

        while (opModeIsActive()){

            if (gamepad2.left_bumper && gamepad2.left_bumper != wasPressed) {
                if (!clawOpen) {
                    claw.position = 0.2
                    clawOpen = true
                } else {
                    claw.position = 0.6
                    clawOpen = false
                }
            }
            wasPressed = gamepad2.left_bumper
        }
    }
}