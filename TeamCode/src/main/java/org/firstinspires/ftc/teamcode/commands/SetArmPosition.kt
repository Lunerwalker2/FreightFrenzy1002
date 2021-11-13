package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Arm
import java.lang.Math.abs

/*
Command that moves the arm autonomously to a given level.

Not really necessary to be a command as the logic is handled by the subsystem, but we have
it here anyway.
 */
class SetArmPosition(private val arm: Arm, private val armPosition: Arm.ArmPosition) : CommandBase() {

    override fun initialize(){
        arm.setArm(armPosition)
        addRequirements(arm)
    }

    override fun isFinished(): Boolean {
        return abs(arm.getArmPosition() - armPosition.targetPosition) <= 40
    }

    override fun end(interrupted: Boolean) {
        arm.stop()
    }
}