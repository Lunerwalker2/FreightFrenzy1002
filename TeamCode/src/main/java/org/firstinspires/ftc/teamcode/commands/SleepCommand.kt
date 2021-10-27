package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

class SleepCommand(private val milliseconds: Long): CommandBase() {


    private val timer = ElapsedTime()


    override fun initialize(){
        timer.reset()
    }

    override fun isFinished() = timer.milliseconds() > milliseconds
}