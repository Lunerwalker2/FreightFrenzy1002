package org.firstinspires.ftc.teamcode.commands.autocommands.cycle

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

open class DropFreightInHubCommand(
        private val drive: SampleMecanumDrive, private val lift: Lift,
        private val scoringArm: ScoringArm, private val bucket: Bucket,
        private val intake: Intake, private val redSide: Boolean) : ParallelCommandGroup() {


    /**
     * This has to be a runtime function since we don't know where we will stop when intaking.
     */
    private fun getTrajectoryCommand(): TrajectorySequence {
        return if(redSide){
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                    .splineToConstantHeading(Vector2d(13.0, -64.5), Math.toRadians(180.0))
                    .splineToConstantHeading(Vector2d(0.0, -58.0), Math.toRadians(160.0))
                    .build()
        } else {
            drive.trajectorySequenceBuilder(drive.poseEstimate)
                    .setReversed(true)
                    .splineToConstantHeading(Vector2d(13.0, 64.5), Math.toRadians(180.0))
                    .splineToConstantHeading(Vector2d(-10.0, 58.0), Math.toRadians(-160.0))
                    .build()
        }
    }

    override fun initialize() {
        addCommands(
                //Drive to the hub and dump at the end, hopefully lift will have extended.
                FollowTrajectorySequenceCommand(drive, getTrajectoryCommand()),
                //Outtake to be safe in case we have other freight in the intake
                SequentialCommandGroup(
                        InstantCommand(intake::outtakeBoth),
                        WaitCommand(2000),
                        InstantCommand(intake::stop)
                ),
                //Move lift out pretty much instantly
                SequentialCommandGroup(
                        WaitCommand(2000),
                        MoveLiftPositionCommand(lift, Lift.Positions.TOP, 5.0, 1800.0, 1700.0),
                ),
                //Try to wait for the lift to extend before moving the arm
                SequentialCommandGroup(
                        WaitCommand(2700),
                        InstantCommand(scoringArm::scoringPosition)
                ),
                SequentialCommandGroup(
                        WaitCommand(3700),
                        InstantCommand(bucket::dump)
                )
        )

        super.initialize()
    }
}