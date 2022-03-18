package org.firstinspires.ftc.teamcode.teleOp

import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
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
@Disabled
@TeleOp(name = "SimpMecTeleOp", group = "TeleOp")
class SimpMecTeleOp : CommandOpMode() {

    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var rightBack: DcMotorEx
    private lateinit var imu: BNO055IMU
    var offset = 0.0
    var prevSlowState = false
    private val matchTimer = ElapsedTime()
    private var isStart = true
    private var endgameRumblePassed = false
    private var intakeHasBeenDetected = false
    private var powerMultiplier = 1.0
    override fun initialize() {
        offset = Extensions.HEADING_SAVER
        telemetry.sendLine("Initializing Subsystems...")
        telemetry.sendLine("Setting bulk cache mode....")
        schedule(BulkCacheCommand(hardwareMap))
        telemetry.sendLine("Setting up gamepads...")
        val driver = GamepadEx(gamepad1)
        val manipulator = GamepadEx(gamepad2)
        telemetry.sendLine("Setting up drive hardware...")
        //Get our motors from the hardware map
        leftFront = hardwareMap.get(DcMotorEx::class.java, "lf")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "lb")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rf")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rb")

        //Create a list for easy iterations that don't take up much room
        val motors = listOf(leftFront, leftBack, rightFront, rightBack)


        motors.forEach {
            //Set all motors to run using encoder, i.e internal velocity control with a 0.85 power cap
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            //Set the zero power behavior to brake, meaning the motor will actively stop
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

        if (isStart) {
            matchTimer.reset()
            isStart = false
        }

        if (!endgameRumblePassed && matchTimer.seconds() > 85) {
            gamepad1.rumble(400)
            gamepad2.rumble(400)
            endgameRumblePassed = true
        }

        val heading = getRobotAngle()

        //If we need to reset our zero angle, increment the offset with the current heading to do so
        if (gamepad1.a && !prevSlowState) {
            offset += heading
            gamepad1.rumble(0.0, 1.0, 300)
        }
        prevSlowState = gamepad1.a


        //Telemetry for most things are handled in the subsystems
        telemetry.addData("Slow Mode Enabled", (powerMultiplier != 0.9))

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

        val mins = (matchTimer.seconds() / 60).toInt()
        val secs = (matchTimer.seconds() % 60).toInt()
        telemetry.addData("Time Remaining", "%d:%d", mins, secs)

        telemetry.update()
    }

    override fun reset() {
        Extensions.HEADING_SAVER = getRobotAngle()
        super.reset()
    }

    //Gets the robot angle in -pi to pi from the imu,
    private fun getRobotAngle(): Double {
        var angle: Double = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle.toDouble()
        angle = AngleUnit.normalizeRadians(angle - offset)
        return angle
    }

}