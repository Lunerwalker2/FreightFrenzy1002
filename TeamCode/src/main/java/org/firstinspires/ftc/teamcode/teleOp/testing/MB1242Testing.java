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

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

@TeleOp
public class MB1242Testing extends LinearOpMode {


    private MB1242 forwardSensor;
    private Rev2mDistanceSensor leftSensor;
    private BNO055IMU imu;

    private boolean redSide = true;

    private final double frontOffset = 0;
    private final double leftOffset = 0;
    private final double rightOffset = 0;

    private double headingOffset = 0;


    private double getRobotAngle(){
        double raw = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        raw -= headingOffset;
        return normalizeRad(raw);
    }

    private static double normalizeRad(double angle){
        while(angle < -Math.PI) angle += 2.0 * Math.PI;
        while(angle >= Math.PI) angle -= 2.0 * Math.PI;
        return angle;
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

            if(gamepad1.dpad_left) redSide = true;
            else if(gamepad1.dpad_right) redSide = false;

            forwardSensor.ping();
            //Wait at least 100ms to allow the sound to dissipate
            sleep(100);

            double forwardReading = forwardSensor.readRange();
            double leftReading = leftSensor.getDistance(DistanceUnit.CM);
            double rightReading = 0.0;
            double heading = getRobotAngle();

            double correctedForwardDistance = forwardReading * Math.cos(heading);
            double correctedLeftDistance = leftReading * Math.cos(heading);
            double correctedRightDistance = rightReading * Math.cos(heading);

            //Make sure the offset of the sensor is changed for each sensor on robot
            double x = frontOffset - correctedForwardDistance;
            //Find the horizontal distance based on what side we are supposed to be on
            double y = (redSide) ? leftOffset - correctedLeftDistance : correctedRightDistance - rightOffset;

            telemetry.addData("Field Side", "%.2f", (redSide) ? "Red" : "Blue");
            telemetry.addData("Robot X (in)", "%.2f", DistanceUnit.INCH.fromCm(x));
            telemetry.addData("Robot Y (in)", DistanceUnit.INCH.fromCm(y));
            telemetry.addData("Robot Heading (rad)/(deg)", "%.2f / %.2f", heading, toDegrees(heading));
            telemetry.addData("Forward Sensor MB1242 Range (cm)", forwardReading);
            telemetry.addData("Left Sensor Rev TOF Range (cm)", leftReading);
            telemetry.addData("Right Sensor Rev TOF Range (cm)", rightReading);
            telemetry.update();
        }
    }
}
