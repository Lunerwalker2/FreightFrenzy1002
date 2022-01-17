package org.firstinspires.ftc.teamcode.teleOp.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo

@Autonomous
@Config
class ScoringArmTest() : LinearOpMode() {

    companion object {
        var armPosition = 0.5
    }


    override fun runOpMode() {

        val scoringArm = hardwareMap.get(Servo::class.java, "scoringArmServo")



        waitForStart()


        while (opModeIsActive()){
            scoringArm.position = armPosition

        }
    }
}