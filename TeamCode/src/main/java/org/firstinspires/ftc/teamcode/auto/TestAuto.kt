package org.firstinspires.ftc.teamcode.auto

import android.graphics.Bitmap
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commands.SetArmPosition
import org.firstinspires.ftc.teamcode.commands.SleepCommand
import org.firstinspires.ftc.teamcode.commands.TestMessageCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.vision.HubLevel
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector

@Autonomous(name="Test command auto")
class TestAuto: AutoBase() {

    lateinit var arm: Arm


    override fun initialize() {
        
        super.initialize()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        arm = Arm(hardwareMap)

        waitForStart()

        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                SleepCommand(2000),
                SetArmPosition(arm, Arm.ArmPosition.BOTTOM_LEVEL)
//                FollowTrajectoryCommand(drive,
//                        drive.trajectoryBuilder(drive.poseEstimate)
//                                .forward(10.0)
//                )
        ))

    }
}
