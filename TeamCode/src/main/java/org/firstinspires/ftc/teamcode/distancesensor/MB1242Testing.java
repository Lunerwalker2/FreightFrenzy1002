package org.firstinspires.ftc.teamcode.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MB1242Testing extends LinearOpMode {


    private MB1242 distanceSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        distanceSensor = hardwareMap.get(MB1242.class, "distanceSensor");

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            distanceSensor.initiateRangeCommand();
            //Wait at least 100ms
            sleep(100);
            telemetry.addData("Range (cm)", distanceSensor.readRangeValueCm());
            telemetry.update();
        }
    }
}
