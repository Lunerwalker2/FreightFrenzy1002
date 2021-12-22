package org.firstinspires.ftc.teamcode.teleOp.testing

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.subsystems.Lift


@Autonomous
class LiftGoToPositionTest() : CommandOpMode() {


    override fun initialize() {
        val lift = Lift(hardwareMap)



    }
}