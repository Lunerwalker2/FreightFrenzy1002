package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.AutoBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.sendLine
import java.lang.Math.toRadians

@Disabled
@Autonomous(name = "Red test cycle")
class TestFastCycleAuto : AutoBase() {


    private lateinit var goToHubStart: TrajectorySequence
    private lateinit var goToWarehouse: TrajectorySequence
    private lateinit var backToHub: TrajectorySequence


    private lateinit var drive: SampleMecanumDrive

    private val startPose = Pose2d(8.34375, -65.375, toRadians(180.0))


    override fun initialize() {

        super.initialize()


        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose


        telemetry.sendLine("Generating trajectories...")



        goToHubStart = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(Vector2d(-10.0, -60.0))
                .waitSeconds(0.3)
                .build()

        goToWarehouse = drive.trajectorySequenceBuilder(goToHubStart.end())
                .setReversed(true)
                .splineToConstantHeading(Vector2d(15.0, -64.0), toRadians(0.0))
                .splineToConstantHeading(Vector2d(50.0, -64.0), toRadians(0.0))
                .build()

        backToHub = drive.trajectorySequenceBuilder(goToWarehouse.end())
                .splineToConstantHeading(Vector2d(15.0, -64.0), toRadians(180.0))
                .splineToConstantHeading(Vector2d(-10.0, -60.0), toRadians(160.0))
                .build()

        telemetry.sendLine("Initializing Subsystems...")


        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                FollowTrajectorySequenceCommand(drive, goToHubStart),
                FollowTrajectorySequenceCommand(drive, goToWarehouse),
                FollowTrajectorySequenceCommand(drive, backToHub),
                FollowTrajectorySequenceCommand(drive, goToWarehouse),
                FollowTrajectorySequenceCommand(drive, backToHub)
        ))


        telemetry.sendLine("Ready for Start!")
    }
}