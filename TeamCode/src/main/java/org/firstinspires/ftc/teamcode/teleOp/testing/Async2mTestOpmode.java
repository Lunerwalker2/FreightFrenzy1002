package org.firstinspires.ftc.teamcode.teleOp.testing;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.testing.AsyncRev2MSensor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp
public class Async2mTestOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor");
        AsyncRev2MSensor asyncSensor = new AsyncRev2MSensor(sensor);
        asyncSensor.setMeasurementIntervalMs(150);
        waitForStart();
        long lastTime = System.currentTimeMillis();
        LynxModule lm = hardwareMap.getAll(LynxModule.class).get(0);
        long timer = System.currentTimeMillis();
        boolean red = false;
        while(opModeIsActive()){
            ArrayList<Blinker.Step> steps = new ArrayList<Blinker.Step>();
            steps.add(new Blinker.Step(red ? Color.RED : Color.BLUE, 10, TimeUnit.MILLISECONDS));
            lm.setPattern(steps);
            red = !red;
            telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Time", asyncSensor.getLastMeasurementTimestamp());
            telemetry.addData("Delay", System.currentTimeMillis() - timer);
            timer = System.currentTimeMillis();
            telemetry.update();
        }
    }
}
