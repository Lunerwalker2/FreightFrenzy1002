package org.firstinspires.ftc.teamcode.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.Intake

/*
I'm using a lot of ftclib classes in this because the framework exists for auto, so might as well.

MOST OF THIS IS NOT HOW THE SDK WORKS PLEASE DON'T TAKE THIS AS AN EXAMPLE OF THAT.
 */
class MecTeleOp : CommandOpMode() {

    lateinit var arm: Arm
    lateinit var intake: Intake
    lateinit var carouselWheel: CarouselWheel

    private var powerMultiplier = 1.0


    //Define an array of the arm stages so that we can increment and decrement the position
    val armPositions: Array<Arm.ArmPosition> = Arm.ArmPosition.values()
    //Hold the current position
    var armCurrentPosition = Arm.ArmPosition.DOWN

    override fun initialize() {

        arm = Arm(hardwareMap)
        intake = Intake(hardwareMap)
        carouselWheel = CarouselWheel(hardwareMap)

        //Define our gamepads with ftclib things
        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)


        //Driver controls
        val slowModeButton = GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER, GamepadKeys.Button.RIGHT_BUMPER) //button to enable slow mode

        //Manipulator Controls TODO: practice with these
        val manipulatorLeftBumper = GamepadButton(manipulator, GamepadKeys.Button.LEFT_BUMPER) //outtake button
        val manipulatorRightBumper = GamepadButton(manipulator, GamepadKeys.Button.RIGHT_BUMPER) //intake button

        val manipulatorDPadUp = GamepadButton(manipulator, GamepadKeys.Button.DPAD_UP) //arm up
        val manipulatorDPadDown = GamepadButton(manipulator, GamepadKeys.Button.DPAD_DOWN) //arm down

        //Carousel wheel
        val manipulatorLeftTrigger = Trigger {gamepad2.left_stick_x > 0.5}
                .whenActive(carouselWheel::forward)
        val manipulatorRightTrigger = Trigger {gamepad2.right_trigger > 0.5}
                .whenActive(carouselWheel::back)

        //Slow mode button
        slowModeButton
                .whenPressed( Runnable { powerMultiplier = 0.6}) //When pressed, set the power to 0.6
                .whenReleased( Runnable { powerMultiplier = 1.0}) //When it's released, set it back to normal


        //outtake
        manipulatorLeftBumper
                .whenPressed(intake::outtake) //Outtake when pressed
                .whenReleased(intake::stop) //Stop when released

        //intake
        manipulatorRightBumper
                .whenPressed(intake::intake) //Intake when pressed
                .whenReleased(intake::stop) //Stop when released

        //arm
        manipulatorDPadUp
                .whenPressed(Runnable {
                    val nextPositionUpNum: Int = armCurrentPosition.ordinal + 1 //Get the next number up from the current
                    if(nextPositionUpNum < armPositions.size){ //Check that it's within the valid positions
                        arm.setArm(armPositions[nextPositionUpNum])  //Set the arm to that position
                        armCurrentPosition = armPositions[nextPositionUpNum]  //Update the current arm position var
                    }
                })

        manipulatorDPadDown
                .whenPressed(Runnable {
                    val nextPositionDownNum: Int = armCurrentPosition.ordinal - 1 //Get the next number down from the current
                    if(nextPositionDownNum >= 0){ //Check that it's within the valid positions
                        arm.setArm(armPositions[nextPositionDownNum]) //Set the arm to that position
                        armCurrentPosition = armPositions[nextPositionDownNum] //Update the current arm position var
                    }
                })

    }

    override fun run() {
        super.run()
    }


}