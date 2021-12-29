package org.firstinspires.ftc.teamcode.teleOp.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.MoveLiftPositionCommand
import org.firstinspires.ftc.teamcode.subsystems.Lift



@Autonomous
class LiftGoToPositionTest() : CommandOpMode() {


    private lateinit var moveUp: MoveLiftPositionCommand
    private lateinit var moveDown: MoveLiftPositionCommand

    private lateinit var lift: Lift

    private var goingUp = true


    override fun initialize() {
        lift = Lift(hardwareMap)

        moveUp = MoveLiftPositionCommand(lift, Lift.Positions.TOP)
        moveDown = MoveLiftPositionCommand(lift, Lift.Positions.IN_ROBOT)



        schedule(SequentialCommandGroup(
                moveUp,
                WaitCommand(1500),
                InstantCommand({ goingUp = false }),
                moveDown,
        ))
    }

    override fun run() {
        super.run()
        val packet = TelemetryPacket()


        packet.put("Target position", if(goingUp) moveUp.controller.setpoint.position else moveDown.controller.setpoint.position)
        packet.put("Current position", lift.getLiftRawPosition())

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

}