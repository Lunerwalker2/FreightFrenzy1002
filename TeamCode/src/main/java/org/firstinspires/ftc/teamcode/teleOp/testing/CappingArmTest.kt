package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.CappingArm
import kotlin.jvm.Throws


@TeleOp
class CappingArmTest : LinearOpMode(){


    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val cappingArm = CappingArm(hardwareMap, telemetry);
        var wasPressed = true
        var currentPosition = CappingArm.Positions.IN_ROBOT.position

        waitForStart()

        if(isStopRequested) return

        while(opModeIsActive()){

            if(gamepad1.dpad_up){
                val nextPos = currentPosition + 0.003
                if (nextPos <= 1) {
                    cappingArm.setArmPositionRaw(nextPos)
                    currentPosition = nextPos
                }
            } else if(gamepad1.dpad_down){
                val nextPos = currentPosition - 0.003
                if (nextPos >= 0) {
                    cappingArm.setArmPositionRaw(nextPos)
                    currentPosition = nextPos
                }
            }

            telemetry.addData("Current Position", currentPosition)
            telemetry.update()

//            wasPressed = gamepad1.dpad_up || gamepad1.dpad_down;
        }


    }
}