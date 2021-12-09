package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.AutoBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.sendLine
import java.lang.Math.toRadians

@Autonomous(name = "Red test cycle")
class TestFastCycleAuto : AutoBase() {


    private lateinit var goToHubStart: TrajectorySequence
    private lateinit var goToWarehouse: TrajectorySequence
    private lateinit var backToHub: TrajectorySequence


    private lateinit var drive: SampleMecanumDrive

    private val startPose = Pose2d(-33.6, -64.0, toRadians(-90.0))


    override fun initialize() {

        super.initialize()


        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose


        telemetry.sendLine("Generating trajectories...")



        goToHubStart = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(Vector2d(-4.5, -40.0), toRadians(112.0))
                .waitSeconds(0.3)
                .setReversed(false)
                .build()

        goToWarehouse = drive.trajectorySequenceBuilder(goToHubStart.end())
                .splineToSplineHeading(Pose2d(9.0, -62.0, toRadians(-10.0)), toRadians(-21.0))
                .splineToSplineHeading(Pose2d(19.0, -63.5, toRadians(0.0)), toRadians(0.0))
                .splineTo(Vector2d(40.0, -63.5), toRadians(0.0))
                .build()

        backToHub = drive.trajectorySequenceBuilder(goToWarehouse.end())
                .setReversed(true)
                .splineToSplineHeading(Pose2d(19.0, -63.5, toRadians(0.0)), toRadians(180.0))
                .splineToSplineHeading(Pose2d(9.0, -62.0, toRadians(-10.0)), toRadians(159.0))
                .splineToSplineHeading(Pose2d(-4.5, -40.0, toRadians(-75.0)), toRadians(105.0))
                .setReversed(false)
                .build()

        telemetry.sendLine("Initializing Subsystems...")


        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                FollowTrajectorySequenceCommand(drive, goToHubStart),
                FollowTrajectorySequenceCommand(drive, goToWarehouse),
                FollowTrajectorySequenceCommand(drive, backToHub)
        ))


        telemetry.sendLine("Ready for Start!")
    }
}