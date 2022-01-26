package org.firstinspires.ftc.teamcode.auto.warehouse

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AutoBase
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@Disabled
@Autonomous
class SimplePark() : AutoBase() {

    private lateinit var drive: SampleMecanumDrive


    private val startPose = Pose2d(8.34375, -65.375, Math.toRadians(180.0)) //left side aligned with left crease


    override fun initialize() {
        super.initialize()

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = startPose




        schedule(
                SequentialCommandGroup(
                        WaitCommand(400),
                        FollowTrajectorySequenceCommand(drive,
                                drive.trajectorySequenceBuilder(startPose)
                                        .forward(30.0)
                                        .build()
                        )
                )
        )
    }
}