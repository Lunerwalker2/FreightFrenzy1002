package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder

class FollowTrajectorySequenceCommand(private val drive: SampleMecanumDrive,
                                      private val trajectorySequence: TrajectorySequence
) : CommandBase() {

    override fun initialize() {
        drive.followTrajectorySequenceAsync(trajectorySequence)
    }

    override fun execute() {
        drive.update()
    }


    override fun isFinished(): Boolean {
        return !drive.isBusy
    }
}