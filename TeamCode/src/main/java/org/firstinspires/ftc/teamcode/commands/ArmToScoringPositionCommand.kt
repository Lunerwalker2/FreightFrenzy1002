package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.old.Arm

class ArmToScoringPositionCommand(private val arm: Arm): CommandBase() {

    private var scoringPosition: Double = 1004.0

    override fun initialize() {
        arm.armPower(0.9)
        addRequirements(arm)
    }


    override fun isFinished(): Boolean {
        return arm.getArmPosition() >= scoringPosition - 50
    }

    override fun end(interrupted: Boolean) {
        arm.armPower(0.0)
    }

}