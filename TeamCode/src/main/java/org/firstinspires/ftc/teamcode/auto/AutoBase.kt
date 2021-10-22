package org.firstinspires.ftc.teamcode.auto

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.hardware.lynx.LynxModule

abstract class AutoBase : CommandOpMode() {

    val allHubs by lazy { hardwareMap.getAll(LynxModule::class.java) }
    
    lateinit var drive: SampleMecanumDrive

    override fun initialize() {
            for (module in allHubs) {
                module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            }

        CommandScheduler.getInstance().addButton {

            // Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
            for (module in allHubs) {
                module.clearBulkCache()
            } }
        
            drive = SampleMecanumDrive(hardwareMap)
    }

}
