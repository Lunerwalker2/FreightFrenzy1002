package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime

/*
Basically does the equivalent of the LinearOpMode's sleep() function.
 */
class SleepCommand(private val milliseconds: Long): CommandBase() {


    private val timer = ElapsedTime()


    override fun initialize(){
        timer.reset()
    }

    override fun isFinished() = timer.milliseconds() > milliseconds
}