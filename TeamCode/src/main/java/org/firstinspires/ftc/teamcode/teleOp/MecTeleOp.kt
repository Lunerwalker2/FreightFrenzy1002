package org.firstinspires.ftc.teamcode.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Intake
import kotlin.math.abs
import kotlin.math.max

/*
I'm using a lot of ftclib classes in this because the framework exists for auto, so might as well.

MOST OF THIS IS NOT HOW THE SDK WORKS PLEASE DON'T TAKE THIS AS AN EXAMPLE OF THAT.
 */
class MecTeleOp : CommandOpMode() {

    lateinit var arm: Arm
//    lateinit var intake: Intake
    lateinit var carouselWheel: CarouselWheel
    lateinit var claw: Claw

    //We could go and use the rr drive class but meh I'd like to show the math anyway
    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightBack: DcMotorEx

    private var powerMultiplier = 1.0


    //Define an array of the arm stages so that we can increment and decrement the position
    val armPositions: Array<Arm.ArmPosition> = Arm.ArmPosition.values()
    //Hold the current position
    var armCurrentPosition = Arm.ArmPosition.DOWN


    override fun initialize() {

        arm = Arm(hardwareMap)
//        intake = Intake(hardwareMap)
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


        //Claw togglex`
        manipulatorLeftBumper.toggleWhenPressed(
                claw::openClaw,
                claw::closeClaw
        )


//        //outtake
//        manipulatorLeftBumper
//                .whenPressed(intake::outtake) //Outtake when pressed
//                .whenReleased(intake::stop) //Stop when released
//
//        //intake
//        manipulatorRightBumper
//                .whenPressed(intake::intake) //Intake when pressed
//                .whenReleased(intake::stop) //Stop when released

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

        val x = -gamepad1.left_stick_y.toDouble()
        val y = gamepad1.left_stick_x.toDouble()
        val rx = gamepad1.right_stick_x.toDouble()

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        val denominator: Double = max(abs(y) + abs(x) + abs(rx), 1.0)

        val frontLeftPower = (y + x + rx) / denominator
        val backLeftPower = (y - x + rx) / denominator
        val frontRightPower = (y - x - rx) / denominator
        val backRightPower = (y + x - rx) / denominator

        leftFront.power = frontLeftPower * powerMultiplier
        leftBack.power = backLeftPower * powerMultiplier
        rightFront.power = frontRightPower * powerMultiplier
        rightBack.power = backRightPower * powerMultiplier

    }


}