package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.lang.Math.toRadians



@Disabled
@Autonomous(name = "Blue Hub & Duck Auto", group = "Hub Auto")
class BlueHubDuckAuto : AutoBase() {

    //Vision
    private val markerDetector = TeamMarkerDetector(hardwareMap)
    private lateinit var hubLevel: HubLevel

    private lateinit var carouselWheel: CarouselWheel
    private lateinit var arm: Arm
    private lateinit var claw: Claw

    //Trajectories for use in auto
    private lateinit var goForward: Trajectory
    private lateinit var turnToHub: TrajectorySequence
    private lateinit var goToHub: Trajectory
    private lateinit var backFromHub: Trajectory
    private lateinit var turnToFaceWall: TrajectorySequence
    private lateinit var goToCarousel: Trajectory
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
                .lineToConstantHeading(Vector2d(-32.0, 40.0))
                .build()

        turnToHub = drive.trajectorySequenceBuilder(goForward.end())
                .turn(toRadians(-135.0))
                .build()

        goToHub = drive.trajectoryBuilder(turnToHub.end())
                .back(6.0)
                .build()

        backFromHub = drive.trajectoryBuilder(goToHub.end())
                .forward(8.0)
                .build()

        turnToFaceWall = drive.trajectorySequenceBuilder(backFromHub.end())
                .turn(toRadians(-45.0))
                .build()

        goToCarousel = drive.trajectoryBuilder(turnToFaceWall.end())
                .lineToLinearHeading(Pose2d(-53.0, 56.5, toRadians(90.0)))
                .build()

        goToStorageUnit = drive.trajectoryBuilder(goToCarousel.end())
                .lineToConstantHeading(Vector2d(-58.0, 35.0))
                .build()


        telemetry.addLine("Initializing Subsystems...")
        telemetry.update()

        arm = Arm(hardwareMap)
        carouselWheel = CarouselWheel(hardwareMap)
        claw = Claw(hardwareMap)


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
                ParallelRaceGroup(
                        WaitCommand(2000),
                        InstantCommand(claw::closeClaw, claw)
                ),
                FollowTrajectoryCommand(drive, goForward),
                FollowTrajectorySequenceCommand(drive, turnToHub),
                FollowTrajectoryCommand(drive, goToHub),
                WaitCommand(500),
                ArmToScoringPositionCommand(arm),
                WaitCommand(500),
                InstantCommand(claw::openClaw, claw),
                WaitCommand(1500),
                ParallelDeadlineGroup(
                        WaitCommand(3000),
                        InstantCommand(claw::closeClaw),
                        InstantCommand({arm.armPower(-0.3)}).whenFinished { arm.armPower(0.0) }
                ),
                FollowTrajectoryCommand(drive, backFromHub),
                FollowTrajectorySequenceCommand(drive, turnToFaceWall),
                FollowTrajectoryCommand(drive, goToCarousel),
                CarouselWheelCommand(carouselWheel, true, 5000),
                FollowTrajectoryCommand(drive, goToStorageUnit)

        ))


    }
}
