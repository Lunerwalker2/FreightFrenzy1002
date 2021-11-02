package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.lang.Math.toRadians



@Autonomous(name = "Blue Duck Auto")
class BlueDuckHubParkAuto : AutoBase() {

    //Vision
    private val markerDetector = TeamMarkerDetector(hardwareMap)
    lateinit var hubLevel: HubLevel


    //Trajectories for use in auto
    lateinit var goForward: Trajectory
    lateinit var goToCarousel: Trajectory
    lateinit var turnRight: TrajectorySequence

    //The RR drive class
    lateinit var drive: SampleMecanumDrive

    //Our starting position
    private val startPose = Pose2d(-42.0, 54.5, toRadians(-90.0))


    override fun initialize() {
        super.initialize()

        //Make sure we set the current position estimate in rr as our starting position
        drive.poseEstimate = startPose

        drive = SampleMecanumDrive(hardwareMap)

        //Generating trajectories is an expensive task, so we do it in init
        goForward = drive.trajectoryBuilder(startPose)
                .forward(30.0)
                .build()

        turnRight = drive.trajectorySequenceBuilder(goForward.end())
                .turn(toRadians(90.0))
                .build()

        goToCarousel = drive.trajectoryBuilder(turnRight.end())
                .lineToConstantHeading(Vector2d(-42.0, 24.5))
                .build()


        //Subsystems
//        val arm = Arm(hardwareMap)


        //Initialize our vision object to get ready for the pipeline
        markerDetector.init()



        //Start the video stream
        markerDetector.startStream()

        //Use an init loop to keep checking the hub level from vision
        while (!isStarted && !isStopRequested) {
            hubLevel = markerDetector.teamMarkerPipeline.hubLevel

            telemetry.addData("Current Hub Level", hubLevel)
            telemetry.update()
        }

        //Make sure we stop the stream
        markerDetector.endStream()

        //Schedule our main program. All of these commands are run during start automatically
        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.addLine("Detected hub level: $hubLevel")
                    telemetry.update()
                }),
                SleepCommand(2000),
                FollowTrajectoryCommand(drive, goForward),
                FollowTrajectorySequenceCommand(drive,turnRight),
                FollowTrajectoryCommand(drive,goToCarousel)
//                FollowTrajectoryCommand(drive,
//                        drive.trajectoryBuilder(drive.poseEstimate)
//                                .lineToConstantHeading(Vector2d(-52.0, 20.0))
//                )
        ))


    }
}
