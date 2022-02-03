package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp
class IntakeSpeedTest(): LinearOpMode(){


    override fun runOpMode() {

        val front = hardwareMap.get(DcMotorSimple::class.java, "frontIntake")
        val back = hardwareMap.get(DcMotorSimple::class.java, "backIntake")

        back.direction = DcMotorSimple.Direction.REVERSE

        var frontSpeed = 0.5
        var backSpeed = 0.5


        waitForStart()


        while(opModeIsActive()){

            if(gamepad2.left_bumper) front.power = frontSpeed
            else front.power = 0.0

            if(gamepad2.right_bumper) back.power = backSpeed
            else back.power = 0.0

            if(gamepad2.dpad_up) {
                frontSpeed += 0.05
                backSpeed += 0.05
            } else if(gamepad2.dpad_down){
                frontSpeed -= 0.05
                backSpeed -= 0.05
            }

            telemetry.addLine("Left bumper for front")
            telemetry.addLine("Right bumper for back")

            telemetry.addData("Front", frontSpeed)
            telemetry.addData("Back", backSpeed)

            telemetry.update()

        }
    }
}