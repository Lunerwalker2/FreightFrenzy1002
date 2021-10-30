package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous(name = "BlueHubDuckPark")
public class BlueHubParkAuto extends LinearOpMode {


    private static final double COUNTS_PER_MOTOR_REV = 560;
    private static final double DRIVE_GEAR_REDUCTION = 1.0; //TODO: find this
    private static final double WHEEL_DIAMETER_INCHES = 3.54; //TODO: find this
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double POSITION_P = 0.05;
    private static final double HEADING_P = 0.05;

    private static final double HEADING_THRESHOLD = 2;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    DcMotorEx arm;

    Servo claw;

    BNO055IMU imu;

    TeamMarkerDetector detector = new TeamMarkerDetector(hardwareMap);

    HubLevel hubLevel = HubLevel.BOTTOM;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        claw = hardwareMap.get(Servo.class, "claw");

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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Start

        if (isStopRequested()) return;

        claw.setPosition(0.2); //TODO: Find this
        sleep(500);

        encoderDrive(0.7, 20, 20, 4); //go forward

        gyroTurn(0.5, -45);

//        encoderDrive(0.5, -8, 8, 3); //turn

        sleep(500);

        arm.setPower(0.2);
        while (arm.getCurrentPosition() < 70) {
            telemetry.addLine("hi");
            telemetry.update();
        }
        arm.setPower(0.05);

        encoderDrive(0.7, 10, 10, 3); //go forward

        //deposit freight
        openClaw();


        sleep(500);

        //back up
        encoderDrive(0.5, -8, -8, 3);

        closeClaw();


        //Let arm drop
        arm.setPower(0);

        //Turn so that we can reverse into the parking zone


    }


    void closeClaw(){
        claw.setPosition(0.7);
    }

    void openClaw(){
        claw.setPosition(0.2);
    }




    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            setTargetPositions(newLeftTarget, newRightTarget);

            // Turn On RUN_TO_POSITION
            setModes(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timer.reset();
            setMotorPowers(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (timer.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setMotorPowers(0,0);

            // Turn off RUN_TO_POSITION
            setModes(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, HEADING_P)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        setMotorPowers(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    void setModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    void setTargetPositions(int leftPosition, int rightPosition) {
        leftFront.setTargetPosition(leftPosition);
        leftBack.setTargetPosition(leftPosition);
        rightFront.setTargetPosition(leftPosition);
        rightBack.setTargetPosition(leftPosition);
    }

    void setMotorPowers(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    double getAngleError(double target, double current) {
        // calculate error in -179 to +180 range  (
        double error = target - current;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }


    double updatePController(double error, double kP, double minOutput, double maxOutput) {
        double output = error * kP; //Find the proportional output

        return Range.clip(output, minOutput, maxOutput); //make sure the returned value is within the bounds
    }
}
