package org.firstinspires.ftc.teamcode.commands.autocommands.duck

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.opmode.FollowerPIDTuner
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder

class ParkInStorageUnitDuck(val drive: SampleMecanumDrive, val redSide: Boolean) : ParallelCommandGroup() {

    init {
        generateTrajectories()
    }

    private lateinit var blueParkTrajectory: TrajectorySequence
    private lateinit var redParkTrajectory: TrajectorySequence

    override fun initialize() {
        addCommands(
                FollowTrajectorySequenceCommand(drive,
                        if(redSide) redParkTrajectory else blueParkTrajectory
                )
        )
    }


    private fun generateTrajectories(){
        blueParkTrajectory = drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(Pose2d(-60.0, 36.0))
                .build()
        redParkTrajectory = drive.trajectorySequenceBuilder(drive.poseEstimate)
                .lineToLinearHeading(Pose2d(-60.0, -36.0))
                .build()
    }


}