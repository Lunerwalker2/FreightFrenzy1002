package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp
class IntakeSensorTest() : LinearOpMode() {


    override fun runOpMode() {

        val colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeSensor");

        colorSensor.initialize() //don't think it's actually needed.


        waitForStart()

        while(opModeIsActive()){

            telemetry.addData("Raw Light Detected", colorSensor.rawLightDetected)
            telemetry.addData("Reported Distance (in)", colorSensor.getDistance(DistanceUnit.INCH))
            telemetry.update()
        }
    }
}