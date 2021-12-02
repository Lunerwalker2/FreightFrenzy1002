package org.firstinspires.ftc.teamcode.teleOp

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel
import org.firstinspires.ftc.teamcode.util.Extensions
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.cubeInput
import kotlin.math.abs
import kotlin.math.max

import org.firstinspires.ftc.teamcode.util.Extensions.Companion.sendLine
import org.firstinspires.ftc.teamcode.util.Extensions.Companion.toFieldRelative

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

    // Buttons/triggers
    private lateinit var leftCarouselTrigger: Trigger
    private lateinit var rightCarouselTrigger: Trigger


    //We could go and use the rr drive class but meh I'd like to show the math anyway
    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightBack: DcMotorEx

    private lateinit var imu: BNO055IMU
    var offset = 0.0

    var prevState = false

    //Drive power multiplier for slow mode
    private var powerMultiplier = 1.0

    private val allHubs by lazy { hardwareMap.getAll(LynxModule::class.java) }


    override fun initialize() {

        offset = Extensions.HEADING_SAVER

        //Extension functions pog see Extensions.kt in util package
        telemetry.sendLine("Initializing Subsystems...")

        carouselWheel = CarouselWheel(hardwareMap, telemetry)

        telemetry.sendLine("Setting bulk cache mode....")
        //Set the bulk read mode to manual
        for (module in allHubs) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //Schedule a clear of the bulk cache each loop
        //This command will reamin scheduled the entire loop
        schedule(RunCommand(
                {
                    for (module in allHubs) {
                        module.clearBulkCache()
                    }
                }
        ))


        telemetry.sendLine("Setting up gamepads...")
        //Define our gamepads with ftclib things
        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)


        //Driver controls
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Runnable {
                    powerMultiplier = 0.6
                })
                .whenReleased(Runnable {
                    powerMultiplier = 1.0
                })

        //Manipulator Controls TODO: practice with these


        //Carousel wheel
        leftCarouselTrigger = Trigger { gamepad2.left_trigger > 0.2 }
                .whileActiveContinuous(Runnable {
                    if (gamepad2.left_trigger < 0.9) carouselWheel.leftForward()
                    else carouselWheel.fastLeftForward()
                })
                .whenInactive(carouselWheel::leftStop)

        rightCarouselTrigger = Trigger { gamepad2.right_trigger > 0.2 }
                .whileActiveContinuous(Runnable {
                    if (gamepad2.right_trigger < 0.9) carouselWheel.rightForward()
                    else carouselWheel.fastRightForward()
                })
                .whenInactive(carouselWheel::rightStop)


        telemetry.sendLine("Setting up drive hardware...")
        //Get our motors from the hardware map
        leftFront = hardwareMap.get(DcMotorEx::class.java, "lf")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "lb")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rf")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rb")


        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

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
            motorConfigurationType.achieveableMaxRPMFraction = 0.95
            it.motorType = motorConfigurationType
        }

        //Reverse the left side motors
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE

        telemetry.sendLine("Ready for start!")
    }

    override fun run() {
        super.run()


        val heading = getRobotAngle()

        if (gamepad1.left_bumper && !prevState) offset += heading
        prevState = gamepad1.left_bumper


        //Telemetry for most things are handled in the subsystems
        telemetry.addData("Slow Mode Enabled", (powerMultiplier != 1.0))


        /* Thanks to FTCLib handling all the things we just did above automatically,
        we barely need to do anything here, except the drive base.
         */

        var y = if (abs(gamepad1.left_stick_y) > 0.05) (-gamepad1.left_stick_y).toDouble() else 0.0 // Remember, this is reversed!
        var x = if (abs(gamepad1.left_stick_x) > 0.05) gamepad1.left_stick_x * 1.1 else 0.0 // Counteract imperfect strafing
        var rx = if (abs(gamepad1.right_stick_x) > 0.05) gamepad1.right_stick_x.toDouble() else 0.0

        y = cubeInput(y, 0.52)
        x = cubeInput(x, 0.52)
        rx = cubeInput(rx, 0.6)


        //Get the field centric inputs
        val pose = toFieldRelative(Pose2d(x, y, rx), heading)

        x = pose.x
        y = pose.y
        rx = pose.heading

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

        telemetry.addLine("Press the left bumper to re-zero the heading.")
        telemetry.addData("Current Heading with offset", AngleUnit.DEGREES.fromRadians(getRobotAngle()))
        telemetry.addData("Offset", AngleUnit.DEGREES.fromRadians(offset))

        telemetry.update()

    }

    private fun getRobotAngle(): Double {
        var angle: Double = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
        angle = AngleUnit.normalizeRadians(angle - offset)
        return angle
    }

}