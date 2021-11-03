package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

/*
Takes an RR trajectory and runs it asynchronously.
 */
class FollowTrajectoryCommand(private val drive: SampleMecanumDrive,
                              private val trajectory: Trajectory
) : CommandBase() {

    //Start the follower
    override fun initialize() {
        drive.followTrajectoryAsync(trajectory)
    }

    //Update our drive powers
    override fun execute() {
        drive.update()
    }


    //End when the trajectory is finished
    override fun isFinished(): Boolean {
        return !drive.isBusy
    }
}