package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

class FollowTrajectoryCommand(private val drive: SampleMecanumDrive,
                              private val trajectory: TrajectoryBuilder
) : CommandBase() {

    override fun initialize() {
        drive.followTrajectoryAsync(trajectory.build())
    }

    override fun execute() {
        drive.update()
    }


    override fun isFinished(): Boolean {
        return !drive.isBusy
    }
}