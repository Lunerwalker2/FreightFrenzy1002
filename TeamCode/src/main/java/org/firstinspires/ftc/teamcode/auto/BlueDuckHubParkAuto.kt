package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.commands.SleepCommand
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.lang.Math.toRadians


@Autonomous(name = "Blue Duck Auto")
class BlueDuckHubParkAuto : AutoBase() {

    lateinit var markerDetector: TeamMarkerDetector
    lateinit var hubLevel: HubLevel


    lateinit var goForward: Trajectory
    lateinit var goToCarusel: Trajectory
    lateinit var turnRight: TrajectorySequence

    val startPose = Pose2d(-42.0, 54.5, toRadians(-90.0))


    override fun initialize() {
        super.initialize()

        goForward = drive.trajectoryBuilder(startPose)
                .forward(30.0)
                .build()

        goToCarusel = drive.trajectoryBuilder(goForward.end())
                .lineToConstantHeading(Vector2d(-22.0, 54.5))
                .build()

        turnRight = drive.trajectorySequenceBuilder(goToCarusel.end())
                .turn(Math.toRadians(90.0))
                .build()



        //Subsystems
//        val arm = Arm(hardwareMap)

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose

        markerDetector = TeamMarkerDetector(hardwareMap)
        markerDetector.init()



        markerDetector.startStream()
        while (!isStarted && !isStopRequested) {
            hubLevel = markerDetector.teamMarkerPipeline.hubLevel

            telemetry.addData("Current Hub Level", hubLevel)
            telemetry.update()
        }

        markerDetector.endStream()

        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.addLine("Detected hub level: $hubLevel")
                    telemetry.update()
                }),
                SleepCommand(2000),
                FollowTrajectoryCommand(drive, goForward),
                FollowTrajectoryCommand(drive,goToCarusel),
                FollowTrajectorySequenceCommand(drive,turnRight)
//                FollowTrajectoryCommand(drive,
//                        drive.trajectoryBuilder(drive.poseEstimate)
//                                .lineToConstantHeading(Vector2d(-52.0, 20.0))
//                )
        ))


    }
}
