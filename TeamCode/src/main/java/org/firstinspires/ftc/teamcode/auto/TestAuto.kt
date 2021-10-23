package org.firstinspires.ftc.teamcode.auto

import android.graphics.Bitmap
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.SleepCommand
import org.firstinspires.ftc.teamcode.commands.TestMessageCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector

@Autonomous(name="test command auto")
class TestAuto: AutoBase() {

    lateinit var markerDetector: TeamMarkerDetector
    lateinit var hubLevel: HubLevel


    override fun initialize() {
        
        super.initialize()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

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
                TestMessageCommand("The test message is here.", 6, telemetry)
//                FollowTrajectoryCommand(drive,
//                        drive.trajectoryBuilder(drive.poseEstimate)
//                                .forward(10.0)
//                )
        ))

    }
}
