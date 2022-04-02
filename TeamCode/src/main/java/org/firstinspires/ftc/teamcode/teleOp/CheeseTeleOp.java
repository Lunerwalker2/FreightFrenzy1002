package org.firstinspires.ftc.teamcode.teleOp;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

@TeleOp(name = "Main TeleOp")
public class CheeseTeleOp extends CommandOpMode {

    private DcMotorEx rightFront, leftFront, rightBack, leftBack, carouselMotor;
    private BNO055IMU imu;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private Lift lift;
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private Servo rightArm, leftArm;
    private double offset = 0.0;
    GamepadEx driver = new GamepadEx(gamepad1);
    GamepadEx manipulator = new GamepadEx(gamepad2);

    // What will run at the start as well as the events
    @Override
    public void initialize() {
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf ");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");

        //initialize subsystems
        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        lift = new Lift(hardwareMap);
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        leftArm = hardwareMap.get(Servo.class, "leftArm");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        telemetry.addLine("Ready to start!");
        telemetry.update();


    }

    // What will run continuously while running
    @Override
    public void run() {
        super.run();

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = orientation.secondAngle; //y axis, on control hub

        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        heading = AngleUnit.normalizeRadians(heading + offset);

        //If reset, set offset to the current ange
        if (gamepad1.x) offset = heading;


        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        //rotate by the heading of the robot
        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
        lx = vector.getX();
        ly = vector.getY();

        double denom = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);

        leftFront.setPower((ly + lx + rx) / denom);
        leftBack.setPower((ly - lx + rx) / denom);
        rightFront.setPower((ly - lx - rx) / denom);
        rightBack.setPower((ly + lx - rx) / denom);

        //intake game elements
        //TODO: make syntax better
        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0 && driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5) {
            rightIntake.setPower(0.3);
        }
        else if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            rightIntake.setPower(1.0);
        }

        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
            leftIntake.setPower(0.3);
        }
        else if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            leftIntake.setPower(1.0);
        }

        //control intake arms
        //TODO: make syntax better
        //TODO: get servo positions
        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER) != true) {
            rightArm.setPosition(0);
        }
        else if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            rightArm.setPosition(1);
        }
        if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER) != true) {
            leftArm.setPosition(0);
        }
        else if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            leftArm.setPosition(1);
        }

        //scoring blocks
        //TODO: make syntax better
        //TODO: get servo positions

        //arm control


        if (manipulator.getButton(GamepadKeys.Button.DPAD_UP)) {
            lift.setLiftPower(0.5);
        }
        else if (manipulator.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            lift.setLiftPower(-0.5);
        }


        //bucket control
        //TODO: make syntax better
        //TODO: get servo positions

        boolean current, previous = false, toggle = false;
        current = manipulator.getButton(GamepadKeys.Button.A);
        if (current && !previous) {
            toggle = !toggle;
        }
        previous = current;

        if (toggle) {
            bucket.close();
        }
        else {
            bucket.open();
        }

        current = manipulator.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        previous = false;
        toggle = false;
        if (current && !previous) {
            toggle = !toggle;
        }
        previous = current;

        if (toggle) {
            scoringArm.scoringPosition();
        }
        else {
            scoringArm.loadingPosition();
        }

        if (manipulator.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            carouselMotor.setPower(0.7);
        }
        else if (manipulator.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            carouselMotor.setPower(-0.7);
        }

        
    }
}
