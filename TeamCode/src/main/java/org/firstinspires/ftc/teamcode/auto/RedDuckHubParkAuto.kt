package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.SleepCommand
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import java.lang.Math.toRadians

@Disabled
@Autonomous(name="Red duck auto")
class RedDuckHubParkAuto : CommandOpMode() {


    lateinit var drive: SampleMecanumDrive

    lateinit var markerDetector: TeamMarkerDetector
    lateinit var hubLevel: HubLevel


    override fun initialize() {
        //Subsystems
//        val arm = Arm(hardwareMap)

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = Pose2d(0.0, 0.0, toRadians(-90.0))

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
//                FollowTrajectoryCommand(drive,
//                        drive.trajectoryBuilder(drive.poseEstimate)
//                                .forward(10.0)
//                )
        ))


    }
}