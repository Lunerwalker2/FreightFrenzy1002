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

@Autonomous(name = "Red test cycle")
class TestFastCycleAuto : CommandOpMode() {


    private lateinit var goToHubStart: Trajectory
    private lateinit var goToWarehouse: TrajectorySequence
    private lateinit var backToHub: TrajectorySequence


    private lateinit var drive: SampleMecanumDrive

    private val startPose = Pose2d(-33.6, -64.0, Math.toRadians(-90.0))


    override fun initialize() {

//        super.initialize()


        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose.copy()


        telemetry.sendLine("Generating trajectories...")

        goToHubStart = drive.trajectoryBuilder(startPose.copy())
                .forward(10.0)
//                .setReversed(true)
//                .splineTo(Vector2d(-4.5, -40.0), Math.toRadians(112.0))
//                .waitSeconds(0.3)
//                .setReversed(false)
//                .splineToSplineHeading(Pose2d(9.0, -62.0, Math.toRadians(-10.0)), Math.toRadians(-21.0))
//                .splineToSplineHeading(Pose2d(19.0, -63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
//                .splineTo(Vector2d(50.0, -63.5), Math.toRadians(0.0))
//                .setReversed(true)
//                .splineToSplineHeading(Pose2d(19.0, -63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
//                .splineToSplineHeading(Pose2d(9.0, -62.0, Math.toRadians(-10.0)), Math.toRadians(159.0))
//                .splineToSplineHeading(Pose2d(-4.5, -40.0, Math.toRadians(-75.0)), Math.toRadians(105.0))
//                .setReversed(false)
                .build()

        goToWarehouse = drive.trajectorySequenceBuilder(goToHubStart.end())
                .waitSeconds(0.1)
                .build()

        backToHub = drive.trajectorySequenceBuilder(goToWarehouse.end())
                .waitSeconds(0.1)
                .build()

        telemetry.sendLine("Initializing Subsystems...")

        val f = 4

        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                FollowTrajectoryCommand(drive, goToHubStart),
//                FollowTrajectorySequenceCommand(drive, goToWarehouse),
//                FollowTrajectorySequenceCommand(drive, backToHub)
        ))


        telemetry.sendLine("Ready for Start!")
    }
}