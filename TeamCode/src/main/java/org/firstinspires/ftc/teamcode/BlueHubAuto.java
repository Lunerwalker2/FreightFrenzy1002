package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous(name="BlueHubDuck")
public class BlueHubAuto extends LinearOpMode {


    private static final double     COUNTS_PER_MOTOR_REV    = 560 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 0.5 ; //TODO: find this
    private static final double     WHEEL_DIAMETER_INCHES   = 3.0 ; //TODO: find this
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double POSITION_P = 0.05;
    private static final double HEADING_P = 0.05;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotorEx arm;

    BNO055IMU imu;

    TeamMarkerDetector detector;

    HubLevel hubLevel = HubLevel.BOTTOM;

    @Override
    public void runOpMode(){

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPositionPIDFCoefficients(5);



        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        detector = new TeamMarkerDetector(hardwareMap);

        detector.initialize();

        detector.startStream();
        while(!isStarted() && opModeIsActive()){
            hubLevel = detector.getTeamMarkerPipeline().getHubLevel();

            telemetry.addData("Hub Level", hubLevel);
            telemetry.update();
        }
        detector.endStream();


        waitForStart();

        //Start

        if (isStopRequested()) return;

        double currentPosition = 0;
        double targetPosition = 0;

        double currentAngle = 0;
        double targetAngle = 0;

        sleep(1000); //wait a bit at the beginning

        targetPosition = 10 * COUNTS_PER_INCH; //set the target as 20 inches ahead
        currentPosition = leftFront.getCurrentPosition();

        while(((targetPosition - currentPosition) > 10) && opModeIsActive()){ //tolerance of 20 encoder ticks
            currentPosition = leftFront.getCurrentPosition(); //get the current position of one of the motors
            double error = targetPosition - currentPosition; //find the error

            double output = updatePController(error, POSITION_P, -0.6, 0.6); //get the output of the p controller
            setMotorPowers(output, output); //apply the power to the motors

        }
        setMotorPowers(0,0); //Stop motors when we reach target

//
//        sleep(500);
//
//            targetAngle = -45; //45 degrees to the left
//            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; //get the current angle
//
//            while (((targetAngle - currentAngle) > 2) && opModeIsActive()) { //tolerance of 2 degrees
//                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; //get the current angle
//                double error = getAngleError(targetAngle, currentAngle);
//
//                double output = updatePController(error, HEADING_P, -0.6, 0.6);
//                setMotorPowers(output, -output);
//            }
//            setMotorPowers(0, 0);
//
//        sleep(500);
//
//        switch (hubLevel){
//            case TOP:
//                arm.setTargetPosition(76); //TODO: Find positions
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.2);
//
//                sleep(1000);
//
//                //TODO: deposit freight
//
//                break;
//            case MIDDLE:
//                arm.setTargetPosition(51);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.2);
//
//                sleep(1000);
//
//                //TODO: deposit freight
//                break;
//            case BOTTOM:
//                arm.setTargetPosition(12);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.2);
//
//                sleep(1000);
//
//                //TODO: deposit freight
//
//                break;
//        }
//
//        //Back away
//
//        targetPosition = 10 * COUNTS_PER_INCH;
//        currentPosition = leftFront.getCurrentPosition();
//
//        while(((targetPosition - currentPosition) > 20) && opModeIsActive()){
//            currentPosition = leftFront.getCurrentPosition();
//            double error = targetPosition - currentPosition;
//
//            double output = updatePController(error, POSITION_P, -1, 0.6);
//            setMotorPowers(output, output);
//
//        }
//        setMotorPowers(0,0);
//
//        //Let arm drop
//        arm.setPower(0);
//
//        //Turn so that we can reverse into the parking zone
//




    }

    void setMotorPowers(double left, double right){
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    double getAngleError(double target, double current){
        // calculate error in -179 to +180 range  (
        double error = target - current;
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;
        return error;
    }


    double updatePController(double error, double kP, double minOutput, double maxOutput){
        double output = error * kP; //Find the proportional output

        return Range.clip(output, minOutput, maxOutput); //make sure the returned value is within the bounds
    }
}
