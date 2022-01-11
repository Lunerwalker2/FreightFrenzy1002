package org.firstinspires.ftc.teamcode.commands.autocommands

import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm
import org.firstinspires.ftc.teamcode.subsystems.Intake
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.MoveLiftPositionCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Bucket
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

class DropFreightInHubCommand(
        private val drive: SampleMecanumDrive, private val lift: Lift,
        private val scoringArm: ScoringArm, private val bucket: Bucket,
        private val intake: Intake, private val redSide: Boolean) : ParallelCommandGroup() {


    /**
     * This has to be a runtime function since we don't know where we will stop when intaking.
     */
    private fun getTrajectoryCommand(): TrajectorySequence =
        drive.trajectorySequenceBuilder(drive.poseEstimate)
                .setReversed(!redSide)
                .splineToConstantHeading(Vector2d(15.0, if (redSide) -64.0 else 64.0), Math.toRadians(180.0))
                .splineToConstantHeading(
                        Vector2d(-10.0,
                                if (redSide) -60.0 else 60.0), Math.toRadians(if (redSide) 160.0 else -160.0))
                .build()

    init {
        addRequirements(scoringArm, bucket, intake)
    }

    override fun initialize() {
        addCommands(
                //Drive to the hub and dump at the end, hopefully lift will have extended.
                FollowTrajectorySequenceCommand(drive, getTrajectoryCommand())
                        .andThen(InstantCommand(bucket::dump)),
                //Outtake to be safe in case we have other freight in the intake
                SequentialCommandGroup(
                        InstantCommand(intake::outtake),
                        WaitCommand(500),
                        InstantCommand(intake::stop)
                ),
                //Move lift out pretty much instantly
                SequentialCommandGroup(
                        WaitCommand(100),
                        MoveLiftPositionCommand(lift, Lift.Positions.TOP, 10.0),
                ),
                //Try to wait for the lift to extend before moving the arm
                SequentialCommandGroup(
                        WaitCommand(800),
                        InstantCommand(scoringArm::scoringPosition)
                )
        )

        super.initialize()
    }
}