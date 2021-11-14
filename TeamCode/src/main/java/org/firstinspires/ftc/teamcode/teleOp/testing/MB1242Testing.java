package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.MB1242;

public class MB1242Testing extends LinearOpMode {


    private MB1242 distanceSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        distanceSensor = hardwareMap.get(MB1242.class, "forwardSensor");

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            distanceSensor.initiateRangeCommand();
            //Wait at least 100ms to allow the sound to dissipate
            sleep(100);
            telemetry.addData("Range (cm)", distanceSensor.readRangeValueCm());
            telemetry.update();
        }
    }
}
