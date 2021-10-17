package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Arm

class SetArmPosition(private val arm: Arm, private val armPosition: Arm.ArmPosition) : CommandBase() {

    override fun initialize(){
        arm.setArm(armPosition)
    }

    override fun isFinished(): Boolean {
        return arm.armState != Arm.ArmState.MOVING_AUTO
    }
}