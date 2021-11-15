package org.firstinspires.ftc.teamcode.auto.duck

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelRaceGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.AutoBase
import org.firstinspires.ftc.teamcode.commands.CarouselWheelCommand
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.old.Claw
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.lang.Math.toRadians


@Autonomous(name = "Blue Duck Auto", group = "Duck Auto")
class BlueDuckAuto : AutoBase() {

    private lateinit var carouselWheel: CarouselWheel
    private lateinit var claw: Claw

    //Trajectories for use in auto
    private lateinit var goForward: Trajectory
    private lateinit var goToCarousel: Trajectory
    private lateinit var turnRight: TrajectorySequence
    private lateinit var goToStorageUnit: Trajectory

    //The RR drive class
    private lateinit var drive: SampleMecanumDrive

    //Our starting position
    private val startPose = Pose2d(-33.6, 64.0, toRadians(-90.0))

    override fun initialize() {
        super.initialize()

        //Make sure we set the current position estimate in rr as our starting position

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose


        telemetry.addLine("Generating trajectories...")
        telemetry.update()

        //Generating trajectories is an expensive task, so we do it in init
        goForward = drive.trajectoryBuilder(startPose)
                .forward(10.0)
                .build()

        turnRight = drive.trajectorySequenceBuilder(goForward.end())
                .turn(toRadians(-90.0))
                .build()

        goToCarousel = drive.trajectoryBuilder(turnRight.end())
                .lineToConstantHeading(Vector2d(-62.0, 61.0))
                .build()

        goToStorageUnit = drive.trajectoryBuilder(goToCarousel.end())
                .lineToConstantHeading(Vector2d(-60.0, 35.0))
                .build()



        telemetry.addLine("Initializing Subsystems...")
        telemetry.update()

        //Subsystems
//        val arm = Arm(hardwareMap)
        carouselWheel = CarouselWheel(hardwareMap)
        claw = Claw(hardwareMap)


        //Schedule our main program. All of these commands are run during start automatically
        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                ParallelRaceGroup(
                        WaitCommand(2000),
                        InstantCommand(claw::closeClaw, claw)
                ),
                FollowTrajectoryCommand(drive, goForward),
                FollowTrajectorySequenceCommand(drive, turnRight),
                FollowTrajectoryCommand(drive, goToCarousel),
                CarouselWheelCommand(carouselWheel, false, 5000),
                FollowTrajectoryCommand(drive, goToStorageUnit),
        ))

        telemetry.addLine("Ready for start!")
        telemetry.update()


    }
}
