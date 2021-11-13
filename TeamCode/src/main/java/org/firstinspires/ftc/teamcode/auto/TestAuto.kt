package org.firstinspires.ftc.teamcode.auto

import android.graphics.Bitmap
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector

@Autonomous(name="Test command auto")
class TestAuto: AutoBase() {

    private lateinit var arm: Arm
    private lateinit var claw: Claw

    private lateinit var drive: SampleMecanumDrive


    override fun initialize() {
        
        super.initialize()

        drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = Pose2d(0.0, 0.0, 0.0)


        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)

        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                WaitCommand(1000),
                InstantCommand(claw::closeClaw, claw),
                WaitCommand(2000),
                FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(Pose2d())
                        .turn(Math.toRadians(90.0))
                        .build()
                ),
                WaitCommand(1000),
                InstantCommand({drive.setMotorPowers(
                        0.8,
                        0.8,
                        0.8,
                        0.8
                )}),
                WaitCommand(2000),
                InstantCommand({drive.setMotorPowers(
                        0.0,
                        0.0,
                        0.0,
                        0.0
                )}),

        ))

    }
}
