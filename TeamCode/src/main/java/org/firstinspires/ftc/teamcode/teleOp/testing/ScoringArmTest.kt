package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo

@Autonomous
class ScoringArmTest() : LinearOpMode() {


    override fun runOpMode() {

        val scoringArm = hardwareMap.get(Servo::class.java, "scoringArmServo")



        waitForStart()

        scoringArm.position = 0.3

        while (opModeIsActive()){


        }
    }
}