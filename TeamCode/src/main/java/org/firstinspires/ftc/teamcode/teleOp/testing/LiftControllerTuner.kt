package org.firstinspires.ftc.teamcode.teleOp.testing

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Config
@TeleOp
class LiftControllerTuner() : LinearOpMode() {

    companion object{
        @JvmField
        var currentPosition = 0.0

        @JvmField
        var targetPosition = 0.0

        @JvmField
        var pidCoefficients = PIDCoefficients(0.05)

        @JvmField
        var staticCof = 0.1
    }

    override fun runOpMode() {

        val liftMotor = hardwareMap.get(DcMotorEx::class.java, "liftMotor")

        var liftController = PIDFController(pidCoefficients, staticCof)

        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        targetPosition = 650.0
        liftController.targetPosition = targetPosition

        if(isStopRequested) return


        while(opModeIsActive()){
            telemetry.addData("Current Position", liftMotor.currentPosition)
            telemetry.addData("Target", liftController.targetPosition)
            telemetry.addData("correction", liftMotor.power)
            telemetry.update()


            currentPosition = liftMotor.currentPosition.toDouble()
            liftMotor.power = liftController.update(currentPosition)

        }
    }
}