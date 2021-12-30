package org.firstinspires.ftc.teamcode.teleOp

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
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
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Lift
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
//    private lateinit var carouselWheel: CarouselWheel
    private lateinit var intake: Intake
    private lateinit var lift: Lift

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
    var prevSlowState = false
    private var prevLiftUp = false
    private var prevLiftDown = false
    private var prevLiftStop = false
    //Drive power multiplier for slow mode
    private var powerMultiplier = 0.9

    override fun initialize() {

        offset = Extensions.HEADING_SAVER

        //Extension functions pog see Extensions.kt in util package
        telemetry.sendLine("Initializing Subsystems...")

//        carouselWheel = CarouselWheel(hardwareMap, telemetry)
        intake = Intake(hardwareMap, telemetry)
        lift = Lift(hardwareMap, telemetry)

        telemetry.sendLine("Setting bulk cache mode....")

        //Schedule a clear of the bulk cache each loop
        //This command will remain scheduled the entire loop
        schedule(BulkCacheCommand(hardwareMap))


        telemetry.sendLine("Setting up gamepads...")
        //Define our gamepads with ftclib things
        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)


        //Driver controls


        //Manipulator Controls TODO: practice with these


        //Carousel wheel
//        leftCarouselTrigger = Trigger { gamepad2.left_trigger > 0.2 }
//                .whileActiveContinuous(Runnable {
//                    if (gamepad2.left_trigger < 0.9) carouselWheel.leftForward()
//                    else carouselWheel.fastLeftForward()
//                })
//                .whenInactive(carouselWheel::leftStop)
//
//        rightCarouselTrigger = Trigger { gamepad2.right_trigger > 0.2 }
//                .whileActiveContinuous(Runnable {
//                    if (gamepad2.right_trigger < 0.9) carouselWheel.rightForward()
//                    else carouselWheel.fastRightForward()
//                })
//                .whenInactive(carouselWheel::rightStop)

        manipulator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(Runnable {
                    if (gamepad2.a) intake.outtake()
                    else intake.intake()
                })
                .whenInactive(intake::stop)

        manipulator.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(
                        Runnable { intake.setSide(false) },
                        Runnable { intake.setSide(true) }
                )

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

        /*
        We do all of this in order to avoid setting the motor power constantly in each case,
        which just adds an extra hardware.
         */
        if(gamepad2.dpad_up && lift.getLiftRawPosition() < 1050){
            if(!prevLiftUp){
                prevLiftUp = true
                prevLiftDown = false
                prevLiftStop = false
                lift.setLiftPower(0.8)
            }
        }
        else if(gamepad2.dpad_down && lift.getLiftRawPosition() > 10){
            if(!prevLiftDown){
                prevLiftDown = true
                prevLiftUp = false
                prevLiftStop = false
                lift.setLiftPower(-0.55)
            }
        }
        else {
            if(!prevLiftStop) {
                prevLiftStop = true
                lift.setLiftPower(0.0)
            }
            prevLiftUp = false
            prevLiftDown = false

        }


        //Set the slow mode one if either bumper is pressed
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
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
        y = cubeInput(y, 0.4)
        x = cubeInput(x, 0.4)
        rx = cubeInput(rx, 0.4)


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