package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.ArmToScoringPositionCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm

@Autonomous(name="Test command auto")
class TestAuto: AutoBase() {

    private lateinit var arm: Arm


    override fun initialize() {
        
        super.initialize()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        arm = Arm(hardwareMap)

        schedule(SequentialCommandGroup(
                InstantCommand({
                    telemetry.addLine("The program started!")
                    telemetry.update()
                }),
                WaitCommand(2000),
                ArmToScoringPositionCommand(arm)
        ))

    }
}
