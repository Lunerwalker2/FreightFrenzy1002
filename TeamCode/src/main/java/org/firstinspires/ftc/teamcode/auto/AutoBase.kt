package org.firstinspires.ftc.teamcode.auto

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

abstract class AutoBase : CommandOpMode() {

    //convenience command for a pause
    fun waitFor(millis: Long): WaitCommand = WaitCommand(millis);

    //start bulk caching
    override fun initialize() {

        //Clear the bulk read cache every iteration
        schedule(BulkCacheCommand(hardwareMap))
    }

    //update the telemetry in the background as well
    override fun run() {
        super.run()

        telemetry.update()
    }

}
