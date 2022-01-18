package org.firstinspires.ftc.teamcode.teleOp

import android.graphics.Color
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.Extensions
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.cubeInput
import kotlin.math.abs
import kotlin.math.max

import org.firstinspires.ftc.teamcode.util.Extensions.Companion.sendLine
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.toFieldRelative
import kotlin.math.sign

/*
I'm using a lot of FTCLib classes in this because the framework exists for auto, so might as well.

This is to prevent the TeleOp file from being huge with all sorts of logic and things. As mentioned
above, the same command and subsystem based framework is already used and exists for autonomous, and
there's no reason to not use it in teleop as well.

MOST OF THIS IS NOT HOW THE SDK WORKS PLEASE DON'T TAKE THIS AS AN EXAMPLE OF THAT.
 */

@TeleOp(name = "Main TeleOp", group = "TeleOp")
class MecTeleOp : CommandOpMode() {


    //Subsystems
    private lateinit var carouselWheel: CarouselWheel
    private lateinit var intake: Intake
    private lateinit var lift: Lift
    private lateinit var scoringArm: ScoringArm
    private lateinit var bucket: Bucket

    //We could go and use the rr drive class but meh I'd like to show the math anyway
    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightBack: DcMotorEx
    private lateinit var imu: BNO055IMU
    var offset = 0.0
    var prevSlowState = false
    private var prevLiftUp = false
    private var prevLiftDown = false
    private var prevLiftStop = false
    //Drive power multiplier for slow mode
    private var powerMultiplier = 0.9
    private val makeReadyToLoadCommand by lazy { MakeReadyToLoadCommand(lift, scoringArm, bucket) }
    private val makeReadyToScoreCommand by lazy { MakeReadyToScoreCommand(lift, scoringArm)}
    private lateinit var manualLiftCommand: ManualLiftCommand

    override fun initialize() {

        offset = Extensions.HEADING_SAVER

        //Extension functions pog see Extensions.kt in util package
        telemetry.sendLine("Initializing Subsystems...")

        carouselWheel = CarouselWheel(hardwareMap, telemetry)
        intake = Intake(hardwareMap, telemetry)
        lift = Lift(hardwareMap, telemetry)
        scoringArm = ScoringArm(hardwareMap, telemetry)
        bucket = Bucket(hardwareMap, telemetry)

        telemetry.sendLine("Setting bulk cache mode....")

        //Schedule a clear of the bulk cache each loop
        //This command will remain scheduled the entire loop
        schedule(BulkCacheCommand(hardwareMap))


        telemetry.sendLine("Setting up gamepads...")
        //Define our gamepads with ftclib things
        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)


        //Driver controls


        //Manipulator Controls


        //Carousel wheel
        Trigger { gamepad2.right_trigger > 0.2 }
                .whileActiveContinuous(Runnable {
                    if (gamepad2.right_trigger < 0.9) carouselWheel.setWheelPower(0.5)
                    else carouselWheel.setWheelPower(0.75)
                })
                .whenInactive(Runnable { carouselWheel.setWheelPower(0.0)})

        manipulator.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(Runnable { carouselWheel.setDirection(false) },
                Runnable { carouselWheel.setDirection(true) })


        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Runnable {
                    if (gamepad1.x) intake.outtake()
                    else intake.intake()
                })
                .whenInactive(intake::stop)


        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(intake::stop)
                .whenPressed(Runnable { intake.setSide(true) })
                .whenPressed(SetHubLEDCommand(hardwareMap, Color.GREEN))

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(intake::stop)
                .whenPressed(Runnable { intake.setSide(false) })
                .whenPressed(SetHubLEDCommand(hardwareMap, Color.RED))


        /* ***********************************************/

        manipulator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        bucket::dump,
                        bucket::load
                )

        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        scoringArm::scoringPosition,
                        scoringArm::loadingPosition
                )

        //Set up the default command for the lift, for more info see the class file of the command
        manualLiftCommand = ManualLiftCommand(
                lift, scoringArm, bucket, manipulator
        )

        //Make this the default lift command
        lift.defaultCommand = PerpetualCommand(manualLiftCommand)

        //If these are scheduled, then the lift command will be interrupted momentarily
        Trigger {-gamepad2.left_stick_y > 0.5}
                .whenActive(makeReadyToScoreCommand)
                .cancelWhenActive(makeReadyToLoadCommand)

        Trigger {-gamepad2.left_stick_y < -0.5}
                .whenActive(makeReadyToLoadCommand)
                .cancelWhenActive(makeReadyToScoreCommand)


        //////////////////////////////////////////////// Drive Base

        telemetry.sendLine("Setting up drive hardware...")
        //Get our motors from the hardware map
        leftFront = hardwareMap.get(DcMotorEx::class.java, "lf")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "lb")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rf")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rb")

        //Create a list for easy iterations that don't take up much room
        val motors = listOf(leftFront, leftBack, rightFront, rightBack)

        //Set all motors to run using encoder, i.e internal velocity control with a 0.85 power cap
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        //Set the zero power behavior to brake, meaning the motor will actively stop
        motors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        //Reverse the left side motors
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE

        telemetry.sendLine("Initializing IMU...")

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        telemetry.sendLine("Ready for start!")
    }

    override fun run() {
        super.run()


        /////////////////////////////////////////////////////////////// Drive Base
        //Set the slow mode one if either bumper is pressed
        if (gamepad1.right_bumper) {
            powerMultiplier = 0.3
        }
        else {
            powerMultiplier = 0.9
        }

        //Store the heading of the robot
        val heading = getRobotAngle()

        //If we need to reset our zero angle, increment the offset with the current heading to do so
        if (gamepad1.a && !prevSlowState) offset += heading
        prevSlowState = gamepad1.a


        //Telemetry for most things are handled in the subsystems
        telemetry.addData("Slow Mode Enabled", (powerMultiplier != 0.9))

        /* Thanks to FTCLib handling all the things we just did above automatically,
        we barely need to do anything here, except the drive base.
         */

        //Check the deadband of the controller
        var y = if (abs(gamepad1.left_stick_y) > 0.02) (-gamepad1.left_stick_y).toDouble() else 0.0 // Remember, this is reversed!
        var x = if (abs(gamepad1.left_stick_x) > 0.02) gamepad1.left_stick_x * 1.1 else 0.0 // Counteract imperfect strafing
        var rx = if (abs(gamepad1.right_stick_x) > 0.02) gamepad1.right_stick_x.toDouble() else 0.0

        //Apply some minor cubing to help with driving
        y = cubeInput(y, 0.2)
        x = cubeInput(x, 0.2)
        rx = cubeInput(rx, 0.2)


        //Get the field centric inputs
        val pose = toFieldRelative(Pose2d(x, y, rx), heading)

        x = pose.x
        y = pose.y
        rx = pose.heading

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        val denominator: Double = max(abs(y) + abs(x) + abs(rx), 1.0)

        val frontLeftPower = (y + x + rx) / denominator
        val backLeftPower = (y - x + rx) / denominator
        val frontRightPower = (y - x - rx) / denominator
        val backRightPower = (y + x - rx) / denominator

        //Finally set the motor powers scaled by the slow mode
        //NEW: Apply a kstatic term to each motor for friction
        leftFront.power = ((sign(frontLeftPower) * 0.03253) + frontLeftPower) * powerMultiplier
        leftBack.power = ((sign(backLeftPower) * 0.03265) + backLeftPower) * powerMultiplier
        rightFront.power = ((sign(frontRightPower) * 0.0433) + frontRightPower) * powerMultiplier
        rightBack.power = ((sign(backRightPower) * 0.0425) + backRightPower) * powerMultiplier

        //Show the current field-centric things of importance.
        telemetry.addLine("Press the left bumper to re-zero the heading.")
        telemetry.addData("Current Heading with offset", AngleUnit.DEGREES.fromRadians(getRobotAngle()))
        telemetry.addData("Offset", AngleUnit.DEGREES.fromRadians(offset))

        telemetry.update()

    }

    //Gets the robot angle in -pi to pi from the imu,
    private fun getRobotAngle(): Double {
        var angle: Double = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
        angle = AngleUnit.normalizeRadians(angle - offset)
        return angle
    }

}