package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
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
import java.lang.Math.toRadians


@Autonomous(name = "Blue duck auto")
class BlueDuckHubParkAuto : CommandOpMode() {


    lateinit var drive: SampleMecanumDrive

    lateinit var markerDetector: TeamMarkerDetector
    lateinit var hubLevel: HubLevel


    override fun initialize() {


        //Subsystems
//        val arm = Arm(hardwareMap)

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = Pose2d(-42.0, 54.5, toRadians(-90.0))

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
                FollowTrajectoryCommand(drive,
                        drive.trajectoryBuilder(drive.poseEstimate)
                                .forward(15.0)
                ),
                FollowTrajectoryCommand(drive,
                        drive.trajectoryBuilder(drive.poseEstimate)
                                .lineTo(Vector2d(-42.0, 45.0))
                ),
                FollowTrajectorySequenceCommand(drive,
                        drive.trajectorySequenceBuilder(drive.poseEstimate)
                                .turn(Math.toRadians(90.0))
                ),
                FollowTrajectoryCommand(drive,
                        drive.trajectoryBuilder(drive.poseEstimate)
                                .lineToConstantHeading(Vector2d(-52.0, 50.0))
                )
        ))


    }
}
