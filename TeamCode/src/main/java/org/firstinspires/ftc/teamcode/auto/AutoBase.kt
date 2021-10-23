package org.firstinspires.ftc.teamcode.auto

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.RunCommand
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

abstract class AutoBase : CommandOpMode() {

    val allHubs by lazy { hardwareMap.getAll(LynxModule::class.java) }



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
