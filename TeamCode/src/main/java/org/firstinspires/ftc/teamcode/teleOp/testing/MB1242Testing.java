package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MB1242;

@TeleOp
public class MB1242Testing extends LinearOpMode {


    private MB1242 forwardSensor;
    private Rev2mDistanceSensor leftSensor;
    private BNO055IMU imu;
    private double offset = 0;


    private double getRobotAngle(){
        double raw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        raw -= offset;
        while(raw < -Math.PI) raw += 2.0 * Math.PI;
        while(raw >= Math.PI) raw -= 2.0 * Math.PI;
        return raw;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        forwardSensor = hardwareMap.get(MB1242.class, "forwardSensor");
        leftSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){
            forwardSensor.ping();
            //Wait at least 100ms to allow the sound to dissipate
            sleep(100);

            double forwardReading = forwardSensor.readRange();
            double leftReading = leftSensor.getDistance(DistanceUnit.CM);
            double heading = getRobotAngle();

            double correctedForwardDistance = forwardReading * Math.cos(heading);
            double correctedLeftDistance = leftReading * Math.cos(heading);

            //Make sure the offset of the sensor is changed for each sensor on robot
            correctedForwardDistance = 72.0 - correctedForwardDistance;
            correctedLeftDistance = 72.0 - correctedLeftDistance;

            telemetry.addData("Robot X (in)", DistanceUnit.INCH.fromCm(correctedForwardDistance));
            telemetry.addData("Robot Y (in)", DistanceUnit.INCH.fromCm(correctedLeftDistance));
            telemetry.addData("Robot Heading (rad)", heading);
            telemetry.addData("Forward Sensor MB1242 Range (cm)", forwardReading);
            telemetry.addData("Left Sensor Rev TOF Range (cm)", leftReading);
            telemetry.update();
        }
    }
}
