package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.Intake

class ManualIntakeCommand(private val intake: Intake, private val driver: GamepadEx) : CommandBase() {


    init {
        addRequirements(intake)

        intake.setFrontFlapDown()
        intake.setBackFlapDown()
    }


    override fun execute() {

        val intakeFront = driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
        val outtakeFront = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3
        val intakeBack = driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        val outtakeBack = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3


        if (intakeFront) {
            intake.setFrontFlapUp()
            intake.intakeBack()
        } else if (outtakeFront) {
            intake.outtakeBack()
            intake.setFrontFlapUp()
        } else {
            intake.stopBack()
            intake.setFrontFlapDown()
        }

        if (intakeBack) {
            intake.setBackFlapUp()
            intake.outtakeFront()
        } else if (outtakeBack) {
            intake.intakeFront()
            intake.setBackFlapUp()
        } else {
            intake.stopFront()
            intake.setBackFlapDown()
        }
    }


    override fun end(interrupted: Boolean) {
        intake.stop()
    }


}