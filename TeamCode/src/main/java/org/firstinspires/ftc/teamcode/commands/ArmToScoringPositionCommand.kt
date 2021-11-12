package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Arm

class ArmToScoringPositionCommand(private val arm: Arm): CommandBase() {

    private var scoringPosition: Double

    init {
        val ticksPerDeg: Double = Arm.getTicksPerRev() / 360.0
        scoringPosition = ticksPerDeg * 165
    }

    override fun initialize() {

        arm.armPower(0.7)

        addRequirements(arm)
    }

    override fun execute() {
        if(arm.getArmPosition() >= scoringPosition - 200){
            arm.armPower(0.5) //slow down a little near the top
        }
    }

    override fun isFinished(): Boolean {
        return arm.getArmPosition() >= scoringPosition - 20
    }

    override fun end(interrupted: Boolean) {
        arm.armPower(0.05)
    }

}