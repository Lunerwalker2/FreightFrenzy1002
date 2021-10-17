package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import java.lang.Math.toRadians

class RedDuckHubParkAuto : CommandOpMode() {


    lateinit var drive: SampleMecanumDrive

    lateinit var markerDetector: TeamMarkerDetector
    lateinit var hubLevel: HubLevel


    override fun initialize() {
        //Subsystems
        val arm = Arm(hardwareMap)

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = Pose2d(0.0, 0.0, toRadians(-90.0))

        markerDetector = TeamMarkerDetector(hardwareMap)
        markerDetector.init()



        markerDetector.startStream()
        while (!isStarted && !isStopRequested) {
            hubLevel = markerDetector.teamMarkerPipeline.hubLevel

            telemetry.addData("Current Hub Level", hubLevel)
            telemetry

        }

        markerDetector.endStream()

        schedule(SequentialCommandGroup(

        ))


    }
}