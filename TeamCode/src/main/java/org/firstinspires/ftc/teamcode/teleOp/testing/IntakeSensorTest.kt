package org.firstinspires.ftc.teamcode.teleOp.testing

import android.graphics.Color
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor

@TeleOp
//@Disabled
class IntakeSensorTest() : LinearOpMode() {


    override fun runOpMode() {

//        val colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeSensor");
        val distanceSensor =
                AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor::class.java, "intakeSensor"))

        distanceSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_SPEED)

        waitForStart()

        while(opModeIsActive()){

            telemetry.addData("Distance (in)", distanceSensor.getDistance(DistanceUnit.INCH))
            telemetry.update()
        }
    }
}