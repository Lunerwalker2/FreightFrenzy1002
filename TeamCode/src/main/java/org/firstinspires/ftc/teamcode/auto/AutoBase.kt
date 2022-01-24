package org.firstinspires.ftc.teamcode.auto

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

abstract class AutoBase : CommandOpMode() {

    fun waitFor(millis: Long): WaitCommand = WaitCommand(millis);

    override fun initialize() {

        //Clear the bulk read cache every iteration
        schedule(BulkCacheCommand(hardwareMap))
    }

    override fun run() {
        super.run()

        telemetry.update()
    }

}
