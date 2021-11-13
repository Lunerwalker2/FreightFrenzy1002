package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

/*
Takes an RR trajectory and runs it asynchronously.
 */
class FollowTrajectoryCommand(private val drive: SampleMecanumDrive,
                              private val trajectory: Trajectory,
                              private val delayAtEndMs: Int = 0
) : CommandBase() {

    private val timer = ElapsedTime()
    private var trajDone = false

    //Start the follower
    override fun initialize() {
        drive.followTrajectoryAsync(trajectory)
    }

    //Update our drive powers
    override fun execute() {
        drive.update()
        if(!drive.isBusy){
            timer.reset()
            trajDone = true
        }
    }


    //End when the trajectory is finished
    override fun isFinished(): Boolean {
        return trajDone && timer.milliseconds() >= delayAtEndMs
    }
}