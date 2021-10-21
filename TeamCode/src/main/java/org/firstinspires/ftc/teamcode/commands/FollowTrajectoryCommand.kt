package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

class FollowTrajectoryCommand(private val drive: SampleMecanumDrive,
                              private val trajectory: Trajectory
) : CommandBase() {

    override fun initialize() {
        drive.followTrajectoryAsync(trajectory)
    }

    override fun execute() {
        drive.update()
    }


    override fun isFinished(): Boolean {
        return !drive.isBusy
    }
}