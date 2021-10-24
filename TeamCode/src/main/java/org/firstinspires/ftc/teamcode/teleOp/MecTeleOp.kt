package org.firstinspires.ftc.teamcode.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake

class MecTeleOp : CommandOpMode() {

    lateinit var arm: Arm
    lateinit var intake: Intake
    lateinit var carouselWheel: CarouselWheel

    private var powerMultiplier = 1.0

    override fun initialize() {

        arm = Arm(hardwareMap)
        intake = Intake(hardwareMap)
        carouselWheel = CarouselWheel(hardwareMap)

        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)


        val slowModeButton = GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER, GamepadKeys.Button.RIGHT_BUMPER) //button to enable slow mode
        slowModeButton.whenPressed( Runnable { powerMultiplier = 0.6})
        slowModeButton.whenReleased( Runnable { powerMultiplier = 1.0})


    }


}