package org.firstinspires.ftc.teamcode.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Intake
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.abs
import kotlin.math.max

/*
I'm using a lot of FTCLib classes in this because the framework exists for auto, so might as well.

This is to prevent the TeleOp file from being huge with all sorts of logic and things. As mentioned
above, the same command and subsystem based framework is already used and exists for autonomous, and
there's no reason to not use it in teleop as well.

MOST OF THIS IS NOT HOW THE SDK WORKS PLEASE DON'T TAKE THIS AS AN EXAMPLE OF THAT.
 */

@TeleOp
class MecTeleOp : CommandOpMode() {

    //Subsystems
    lateinit var arm: Arm
//    lateinit var intake: Intake
    lateinit var carouselWheel: CarouselWheel
    lateinit var claw: Claw

    //We could go and use the rr drive class but meh I'd like to show the math anyway
    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightBack: DcMotorEx

    //Drive power multiplier for slow mode
    private var powerMultiplier = 1.0


    //Define an array of the arm stages so that we can increment and decrement the position
    val armPositions: Array<Arm.ArmPosition> = Arm.ArmPosition.values()

    //Hold the current position of the arm
    var armCurrentPosition = Arm.ArmPosition.DOWN


    override fun initialize() {

//        arm = Arm(hardwareMap)
//        claw = Claw(hardwareMap)
//        carouselWheel = CarouselWheel(hardwareMap)
//        intake = Intake(hardwareMap)


        //Define our gamepads with ftclib things
        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)


        //Driver controls
        val slowModeButton = GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER) //button to enable slow mode

        //Manipulator Controls TODO: practice with these
        val manipulatorLeftBumper = GamepadButton(manipulator, GamepadKeys.Button.LEFT_BUMPER) //outtake button
        val manipulatorRightBumper = GamepadButton(manipulator, GamepadKeys.Button.RIGHT_BUMPER) //intake button

        val manipulatorDPadUp = GamepadButton(manipulator, GamepadKeys.Button.DPAD_UP) //arm up
        val manipulatorDPadDown = GamepadButton(manipulator, GamepadKeys.Button.DPAD_DOWN) //arm down

//        //Carousel wheel
//        val manipulatorLeftTrigger = Trigger {gamepad2.left_stick_x > 0.5}
//                .whenActive(carouselWheel::forward)
//        val manipulatorRightTrigger = Trigger {gamepad2.right_trigger > 0.5}
//                .whenActive(carouselWheel::back)

        //Slow mode button
        slowModeButton
                .whenPressed( Runnable { powerMultiplier = 0.6}) //When pressed, set the power to 0.6
                .whenReleased( Runnable { powerMultiplier = 1.0}) //When it's released, set it back to normal


//        //Claw toggles
//        manipulatorLeftBumper.toggleWhenPressed(
//                claw::openClaw,
//                claw::closeClaw
//        )


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
//                        arm.setArm(armPositions[nextPositionUpNum])  //Set the arm to that position
                        armCurrentPosition = armPositions[nextPositionUpNum]  //Update the current arm position var
                    }
                })

        manipulatorDPadDown
                .whenPressed(Runnable {
                    val nextPositionDownNum: Int = armCurrentPosition.ordinal - 1 //Get the next number down from the current
                    if(nextPositionDownNum >= 0){ //Check that it's within the valid positions
//                        arm.setArm(armPositions[nextPositionDownNum]) //Set the arm to that position
                        armCurrentPosition = armPositions[nextPositionDownNum] //Update the current arm position var
                    }
                })

        //Get our motors from the hardware map
        leftFront = hardwareMap.get(DcMotorEx::class.java, "lf")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "lb")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rf")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rb")

        //Create a list for easy iterations that don't take up much room
        val motors = listOf(leftFront, leftBack, rightFront, rightBack)

        //Set all motors to run using encoder, i.e internal velocity control with a 0.85 power cap
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        //Set the zero power behavior to brake, meaning the motor will actively stop
        motors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        /*
        We don't need to do this, however since we can it can help. Basically, the run using
        encoders runmode will control the velocity using the internal motor encoder and a PIDF
        controller, however it determines this velocity using the fraction of the motor's theoretical
        maximum velocity. Two things are important here, the first being that the SDK doesn't know the
        actual maximum velocity of the motor (different gearboxes), so it uses a guess on the value.
        For example, a motor configured as a gb 5202 motor might have the max velocity configured for
        a 5202 1:19.2. The second thing is that the SDK caps the max velocity at 85% to ensure enough
        head room for adjustments. We can change the first one if we want to and we have the empirical
        maximum velocity of our specific motors. We can change the second one easily.
         */
        motors.forEach {
            val motorConfigurationType: MotorConfigurationType = it.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 0.9
            it.motorType = motorConfigurationType
        }

        //Reverse the left side motors
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE

    }

    override fun run() {
        super.run()


        telemetry.addData("Slow Mode Enabled", (powerMultiplier != 1.0))
        telemetry.addData("Arm Level: ", armCurrentPosition)
        telemetry.update()

        /* Thanks to FTCLib handling all the things we just did above automatically,
        we barely need to do anything here, except the drive base.
         */



        val y = -gamepad1.left_stick_y.toDouble()
        val x = gamepad1.left_stick_x.toDouble()
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