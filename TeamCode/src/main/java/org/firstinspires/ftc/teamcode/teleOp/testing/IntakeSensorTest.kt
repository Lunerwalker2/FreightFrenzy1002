package org.firstinspires.ftc.teamcode.teleOp.testing

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp
//@Disabled
class IntakeSensorTest() : LinearOpMode() {


    override fun runOpMode() {

        val colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeSensor");

        colorSensor.initialize() //don't think it's actually needed.


        waitForStart()

        while(opModeIsActive()){

            val colors: NormalizedRGBA = colorSensor.normalizedColors

            val hsv = FloatArray(3)
            Color.colorToHSV(colors.toColor(), hsv)

            telemetry.addData("Raw Light Detected", colorSensor.rawLightDetected)
            telemetry.addData("Light Detected", colorSensor.lightDetected)
            telemetry.addData("Reported Distance (in)", colorSensor.getDistance(DistanceUnit.INCH))
            telemetry.addData("H (0-360)", hsv[0])
            telemetry.addData("S (0-1)", hsv[1])
            telemetry.addData("V (0-1)", hsv[2])
            telemetry.update()
        }
    }
}