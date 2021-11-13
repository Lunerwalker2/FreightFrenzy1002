package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Arm

class ArmToScoringPositionCommand(private val arm: Arm): CommandBase() {

    private var scoringPosition: Double = 948.0

    override fun initialize() {
        arm.armPower(0.7)
        addRequirements(arm)
    }


    override fun isFinished(): Boolean {
        return arm.getArmPosition() >= scoringPosition - 100
    }

    override fun end(interrupted: Boolean) {
        arm.armPower(0.0)
    }

}