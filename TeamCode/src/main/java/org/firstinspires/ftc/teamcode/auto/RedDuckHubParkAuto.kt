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


/*
 * Lots of comments are omitted from this class mainly because they are/would be mostly the same as the
 * corresponding blue alliance one.
 *
 * The blue alliance autos are where we are usually doing dev since as of now that's the only side
 * we have taped out, so the red autos will be a bit behind with changes.
 */
@Disabled
@Autonomous(name="Red Hub & Duck Auto")
class RedDuckHubParkAuto : AutoBase() {


    lateinit var markerDetector: TeamMarkerDetector
    lateinit var hubLevel: HubLevel

    lateinit var drive: SampleMecanumDrive


    override fun initialize() {
        super.initialize()
        drive = SampleMecanumDrive(hardwareMap)

        //Subsystems
//        val arm = Arm(hardwareMap)

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
                SleepCommand(2000)
//                FollowTrajectoryCommand(drive,
//                        drive.trajectoryBuilder(drive.poseEstimate)
//                                .forward(10.0)
//                )
        ))


    }
}
