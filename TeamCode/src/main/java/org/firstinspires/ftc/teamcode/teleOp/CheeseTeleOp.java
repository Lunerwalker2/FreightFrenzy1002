package org.firstinspires.ftc.teamcode.teleOp;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

import java.util.List;

@TeleOp(name = "Main TeleOp")
public class CheeseTeleOp extends CommandOpMode {

    private DcMotorEx rightFront, leftFront, rightBack, leftBack;
    private BNO055IMU imu;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
    private Lift lift;
    private CarouselWheel carouselWheel;
    private double offset = 0.0;

    private MoveLiftToScoringPositionCommand moveLiftToScoringPositionCommand;
    private MoveLiftToLoadingPositionCommand moveLiftToLoadingPositionCommand;
    private ManualLiftCommand manualLiftCommand;


    // What will run at the start as well as the events
    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf ");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");

        //initialize subsystems
        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        leftIntake = new LeftIntake(hardwareMap);
        rightIntake = new RightIntake(hardwareMap);
        lift = new Lift(hardwareMap);
        carouselWheel = new CarouselWheel(hardwareMap);

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

        moveLiftToScoringPositionCommand = new MoveLiftToScoringPositionCommand(
                lift, scoringArm, bucket
        );

        moveLiftToLoadingPositionCommand = new MoveLiftToLoadingPositionCommand(
                lift, scoringArm, bucket
        );


        //Carousel wheel
        new Trigger(() -> gamepad2.right_trigger > 0.2)
                .whileActiveContinuous(() -> {
                    if (gamepad2.right_trigger < 0.8) carouselWheel.setWheelPower(0.52);
                    else carouselWheel.setWheelPower(0.8);
                })
                .whenInactive(() -> {
                    carouselWheel.setWheelPower(0.0);
                });

        manipulator.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(() -> {
                            carouselWheel.setDirection(false);
                        },
                        () -> {
                            carouselWheel.setDirection(true);
                        });

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(leftIntake::intakeDown, leftIntake::intakeUp);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(rightIntake::intakeDown, rightIntake::intakeUp);

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    if (!leftIntake.up) leftIntake.intake();
                })
                .whenInactive(leftIntake::stop);

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    if (!rightIntake.up) rightIntake.intake();
                })
                .whenInactive(rightIntake::stop);

        manualLiftCommand = new ManualLiftCommand(lift, manipulator);

        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    if (!scoringArm.loading) {
                        bucket.open();
                    }
                })
                .whenInactive(() -> {
                    if (!scoringArm.loading) {
                        bucket.close();
                    }
                });

        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(
                        () -> {
                            scoringArm.scoringPosition();
                            bucket.close();
                        },
                        () -> {
                            scoringArm.loadingPosition();
                            bucket.open();
                        }
                );

        new Trigger(() -> manipulator.getLeftY() > 0.5)
                .whenActive(moveLiftToScoringPositionCommand)
                .cancelWhenActive(moveLiftToLoadingPositionCommand);

        new Trigger(() -> manipulator.getLeftY() < -0.5)
                .whenActive(moveLiftToLoadingPositionCommand)
                .cancelWhenActive(moveLiftToScoringPositionCommand);


    }

    // What will run continuously while running
    @Override
    public void run() {
        super.run();

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = orientation.secondAngle; //y axis, on control hub
        telemetry.addData("Z axis", orientation.firstAngle);
        telemetry.addData("Y axis", orientation.secondAngle);
        telemetry.addData("X axis", orientation.thirdAngle);

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

        List<LynxModule> f = hardwareMap.getAll(LynxModule.class);
        telemetry.addData("Hub 1", f.get(0).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Hub 2", f.get(1).getCurrent(CurrentUnit.AMPS));


        //intake game elements
        //TODO: make syntax better
//        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0 && driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5) {
//            rightIntake.setPower(0.3);
//        }
//        else if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
//            rightIntake.setPower(1.0);
//        }
//
//        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 && driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
//            leftIntake.setPower(0.3);
//        }
//        else if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
//            leftIntake.setPower(1.0);
//        }

        //control intake arms
        //TODO: make syntax better
        //TODO: get servo positions
//        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER) != true) {
//            rightArm.setPosition(0);
//        }
//        else if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
//            rightArm.setPosition(1);
//        }
//        if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER) != true) {
//            leftArm.setPosition(0);
//        }
//        else if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
//            leftArm.setPosition(1);
//        }

        //scoring blocks
        //TODO: make syntax better
        //TODO: get servo positions

        //arm control

//
//        if (manipulator.getButton(GamepadKeys.Button.DPAD_UP)) {
//            lift.setPower(0.5);
//        }
//        else if (manipulator.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            lift.setPower(-0.5);
//        }


        //bucket control
        //TODO: make syntax better
        //TODO: get servo positions

//        boolean current, previous = false, toggle = false;
//        current = manipulator.getButton(GamepadKeys.Button.A);
//        if (current && !previous) {
//            toggle = !toggle;
//        }
//        previous = current;
//
//        if (toggle) {
//            bucket.close();
//        }
//        else {
//            bucket.open();
//        }
//
//        current = manipulator.getButton(GamepadKeys.Button.RIGHT_BUMPER);
//        previous = false;
//        toggle = false;
//        if (current && !previous) {
//            toggle = !toggle;
//        }
//        previous = current;
//
//        if (toggle) {
//            scoringArm.scoringPosition();
//        }
//        else {
//            scoringArm.loadingPosition();
//        }
//
//        if (manipulator.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
//            carouselMotor.setPower(0.7);
//        }
//        else if (manipulator.getButton(GamepadKeys.Button.DPAD_LEFT)) {
//            carouselMotor.setPower(-0.7);
//        }
        telemetry.update();
    }
}
