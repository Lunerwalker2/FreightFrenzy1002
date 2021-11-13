package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.Arm

class ArmToScoringPositionCommand(private val arm: Arm, private val telemetry: Telemetry? = null): CommandBase() {

    private var scoringPosition: Double = 948.0

    override fun initialize() {
        arm.armPower(0.7)
        addRequirements(arm)
    }

    override fun execute() {
        telemetry?.addData("current position", arm.getArmPosition())
        telemetry?.update()
    }


    override fun isFinished(): Boolean {
        return arm.getArmPosition() >= (scoringPosition - 20.0)
    }

    override fun end(interrupted: Boolean) {
        arm.stop()
    }

}