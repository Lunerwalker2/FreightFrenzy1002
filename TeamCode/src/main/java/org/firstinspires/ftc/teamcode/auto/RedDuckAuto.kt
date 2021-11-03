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



@Autonomous(name = "Red Duck Auto")
class RedDuckAuto : AutoBase() {


    //Trajectories for use in auto
    lateinit var goForward: Trajectory
    lateinit var goToCarousel: Trajectory
    lateinit var turnLeft: TrajectorySequence

    //The RR drive class
    lateinit var drive: SampleMecanumDrive

    //Our starting position
    private val startPose = Pose2d(-42.0, -54.5, toRadians(-90.0))


    override fun initialize() {
        super.initialize()

        //Make sure we set the current position estimate in rr as our starting position
        drive.poseEstimate = startPose

        drive = SampleMecanumDrive(hardwareMap)

        telemetry.addLine("Generating trajectories...")
        telemetry.update()


        //Generating trajectories is an expensive task, so we do it in init
        goForward = drive.trajectoryBuilder(startPose)
                .forward(10.0)
                .build()

        turnLeft = drive.trajectorySequenceBuilder(goForward.end())
                .turn(-90.0)
                .build()

        goToCarousel = drive.trajectoryBuilder(turnLeft.end())
                .lineToConstantHeading(Vector2d(-62.0, -44.5))
                .build()


        telemetry.addLine("Initializing Subsystems...")
        telemetry.update()

        //Subsystems
//        val arm = Arm(hardwareMap)


        //Schedule our main program. All of these commands are run during start automatically
        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                SleepCommand(2000),
                FollowTrajectoryCommand(drive, goForward),
                FollowTrajectorySequenceCommand(drive, turnLeft),
                FollowTrajectoryCommand(drive,goToCarousel)
        ))

        telemetry.addLine("Ready for start!")
        telemetry.update()

    }
}
