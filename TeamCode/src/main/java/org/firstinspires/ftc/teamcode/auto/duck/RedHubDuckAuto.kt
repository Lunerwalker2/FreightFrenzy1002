package org.firstinspires.ftc.teamcode.auto.duck

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.auto.AutoBase
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.old.Arm
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.old.Claw
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.sendLine
import java.lang.Math.toRadians


/*
 * Lots of comments are omitted from this class mainly because they are/would be mostly the same as the
 * corresponding blue alliance one.
 *
 * The blue alliance autos are where we are usually doing dev since as of now that's the only side
 * we have taped out, so the red autos will be a bit behind with changes.
 */
//@Disabled
@Autonomous(name = "Red Hub & Duck Auto", group = "Hub Auto")
class RedHubDuckAuto : AutoBase() {

    //Vision
    private lateinit var markerDetector: TeamMarkerDetector
    private var hubLevel: HubLevel = HubLevel.TOP

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
    private val startPose = Pose2d(-33.6, -64.0, toRadians(90.0))


    override fun initialize() {
        super.initialize()

        //Make sure we set the current position estimate in rr as our starting position


        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose


        telemetry.sendLine("Generating trajectories...")

        //Generating trajectories is an expensive task, so we do it in init
        goForward = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(Vector2d(-32.0, -40.0))
                .build()

        turnToHub = drive.trajectorySequenceBuilder(goForward.end())
                .turn(toRadians(135.0))
                .build()

        goToHub = drive.trajectoryBuilder(turnToHub.end())
                .back(6.0)
                .build()

        backFromHub = drive.trajectoryBuilder(goToHub.end())
                .forward(4.0)
                .build()

        turnToFaceWall = drive.trajectorySequenceBuilder(backFromHub.end())
                .turn(toRadians(45.0))
                .build()

        goToCarousel = drive.trajectoryBuilder(turnToFaceWall.end())
                .lineToLinearHeading(Pose2d(-64.0, -63.0, toRadians(-90.0)))
                .build()

        goToStorageUnit = drive.trajectoryBuilder(goToCarousel.end())
                .lineToConstantHeading(Vector2d(-63.0, -35.0))
                .build()


        telemetry.sendLine("Initializing Subsystems...")

        arm = Arm(hardwareMap)
        carouselWheel = CarouselWheel(hardwareMap)
        claw = Claw(hardwareMap)


        markerDetector = TeamMarkerDetector(hardwareMap)
        /*
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

        */

        //Schedule our main program. All of these commands are run during start automatically
        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.addLine("Detected hub level: $hubLevel")
                    telemetry.update()
                }),
                InstantCommand(claw::closeClaw, claw).alongWith(waitFor(2000)),
                FollowTrajectoryCommand(drive, goForward).andThen(waitFor(200)),
                FollowTrajectorySequenceCommand(drive, turnToHub).andThen(waitFor(200)),
                FollowTrajectoryCommand(drive, goToHub).andThen(waitFor(500)),
                ArmToScoringPositionCommand(arm).withTimeout(3000),
                InstantCommand(claw::openClaw, claw).alongWith(waitFor(1500)),
                InstantCommand(claw::closeClaw, claw),
                InstantCommand({ arm.armPower(-0.5) }, arm).andThen(waitFor(2000)),
                InstantCommand({ arm.armPower(0.0) }, arm),
                FollowTrajectoryCommand(drive, backFromHub),
                FollowTrajectorySequenceCommand(drive, turnToFaceWall),
                FollowTrajectoryCommand(drive, goToCarousel).andThen(waitFor(100)),
                CarouselWheelCommand(carouselWheel, false,false).withTimeout(5000),
                FollowTrajectoryCommand(drive, goToStorageUnit)
        ))

        telemetry.sendLine("Ready for start!")

    }
}
