package org.firstinspires.ftc.teamcode.auto

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

abstract class AutoBase : CommandOpMode() {

    /*
    Create a variable to hold the list of hubs on the robot for use of bulk caching.

    By lazy means that the object will be created the first time it is accessed, i.e, when
    the hardware map is set in init.
     */
    private val allHubs: List<LynxModule> by lazy { hardwareMap.getAll(LynxModule::class.java) }

    fun waitFor(millis: Long): WaitCommand = WaitCommand(millis);

    override fun initialize() {
        //Set the mode to manual since we can clear this every loop (see the caching ex. in the samples)
        for (module in allHubs) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //Clear the bulk read cache every iteration
        CommandScheduler.getInstance().schedule(RunCommand(
                {
                    for (module in allHubs) {
                        module.clearBulkCache()
                    }
                }
        ))
    }

}
