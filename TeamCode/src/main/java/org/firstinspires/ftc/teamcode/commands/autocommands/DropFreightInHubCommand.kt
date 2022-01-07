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


    private val trajectoryCommand: TrajectorySequence
        get() = drive.trajectorySequenceBuilder(drive.poseEstimate)
                .setReversed(!redSide)
                .splineToConstantHeading(Vector2d(15.0, if (redSide) -64.0 else 64.0), Math.toRadians(180.0))
                .splineToConstantHeading(
                        Vector2d(-10.0,
                                if (redSide) -60.0 else 60.0), Math.toRadians(if (redSide) 160.0 else -160.0))
                .build()

    init {

        addRequirements(lift, scoringArm, bucket, intake)

        addCommands(
                FollowTrajectorySequenceCommand(drive, trajectoryCommand)
                        .andThen(InstantCommand(bucket::dump)),
                SequentialCommandGroup(
                        InstantCommand(intake::outtake),
                        WaitCommand(500),
                        InstantCommand(intake::stop)
                ),
                SequentialCommandGroup(
                        WaitCommand(300),
                        MoveLiftPositionCommand(lift, Lift.Positions.TOP, 10.0),
                        InstantCommand(scoringArm::scoringPosition)
                )
        )
    }
}