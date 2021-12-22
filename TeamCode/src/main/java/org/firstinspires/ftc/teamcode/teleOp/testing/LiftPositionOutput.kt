package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp
class LiftPositionOutput() : LinearOpMode() {


    override fun runOpMode() {

        val liftMotor = hardwareMap.get(DcMotorEx::class.java, "liftMotor")

        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


        waitForStart()

        if(isStopRequested) return


        while(opModeIsActive()){
            telemetry.addData("Current Position", liftMotor.currentPosition)
            telemetry.update()

        }
    }
}