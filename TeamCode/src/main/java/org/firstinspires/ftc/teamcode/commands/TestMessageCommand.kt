package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.robotcore.external.Telemetry

/*
Sends a message to telemetry a certain number of times, for testing purposes.
 */
class TestMessageCommand(private val message: String, private val times: Int, private val telemetry: Telemetry): CommandBase() {

    private var count = 0

    override fun initialize(){
        telemetry.addLine(message)
    }

    override fun execute() {
        telemetry.addLine("Executed $count times!")
        count++
    }

    override fun end(interrupted: Boolean) {
        telemetry.update()
    }


    override fun isFinished(): Boolean {
        return (count > times)
    }

}