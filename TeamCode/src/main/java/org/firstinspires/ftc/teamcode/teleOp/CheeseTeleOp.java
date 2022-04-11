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

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToMidScoringPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.util.Extensions;

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
    private double powerMultiplier = 1.0;
    private boolean prevSlowState = false;

    private MoveLiftToScoringPositionCommand moveLiftToScoringPositionCommand;
    private MoveLiftToMidScoringPositionCommand moveLiftToMidScoringPositionCommand;
    private MoveLiftToLoadingPositionCommand moveLiftToLoadingPositionCommand;
    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;


    // What will run at the start as well as the events
    @Override
    public void initialize() {

        schedule(new BulkCacheCommand(hardwareMap));

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

        moveLiftToMidScoringPositionCommand = new MoveLiftToMidScoringPositionCommand(
                lift, scoringArm, bucket
        );

        moveLiftToLoadingPositionCommand = new MoveLiftToLoadingPositionCommand(
                lift, scoringArm, bucket
        );

        manualLiftCommand = new ManualLiftCommand(lift, manipulator);

        manualLiftResetCommand = new ManualLiftResetCommand(lift, manipulator);


        //Carousel wheel
        new Trigger(() -> gamepad2.right_trigger > 0.2)
                .whileActiveContinuous(() -> {
                    if (gamepad2.right_trigger < 0.8) carouselWheel.setWheelPower(0.55);
                    else carouselWheel.setWheelPower(0.7);
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
                .whileActiveContinuous(() -> {
                    if (leftIntake.up) leftIntake.intakePower(0.2);
                    else leftIntake.intake();
                })
                .whenInactive(leftIntake::stop);

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(() -> {
                    if (rightIntake.up) rightIntake.intakePower(0.2);
                    else rightIntake.intake();
                })
                .whenInactive(rightIntake::stop);



        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

        //If the trigger is pressed and the scoring arm is not in the robot then open the bucket
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
                            scoringArm.setPosition(0.75);
                            scoringArm.loading = false;
                            bucket.close();
                        },
                        () -> {
                            scoringArm.loadingPosition();
                            bucket.open();
                        }
                );

        new Trigger(() -> manipulator.getLeftY() > 0.5)
                //Automatic extend and slow drive base
                .whenActive(moveLiftToScoringPositionCommand)
                .whenActive(() -> powerMultiplier = 0.5)
                .cancelWhenActive(moveLiftToLoadingPositionCommand);

        new Trigger(() -> manipulator.getRightY() < -0.5)
                //Automatic extend and slow drive base
                .whenActive(moveLiftToMidScoringPositionCommand)
                .whenActive(() -> powerMultiplier = 0.5)
                .cancelWhenActive(moveLiftToLoadingPositionCommand);

        new Trigger(() -> manipulator.getLeftY() < -0.5 || manipulator.getRightY() > 0.5)
                //Automatic retract and restore drive base
                .whenActive(moveLiftToLoadingPositionCommand)
                .whenActive(() -> powerMultiplier = 1.0)
                .cancelWhenActive(moveLiftToScoringPositionCommand)
                .cancelWhenActive(moveLiftToMidScoringPositionCommand);

        //Starts when pressed and ended when released
        manipulator.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(manualLiftResetCommand);

        //Just in case something is off, give a reset for this
        manipulator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> powerMultiplier = 1.0);

    }

    // What will run continuously while running
    @Override
    public void run() {
        super.run();

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = orientation.firstAngle; //y axis, on control hub
        telemetry.addData("Z axis", orientation.firstAngle);
        telemetry.addData("Y axis", orientation.secondAngle);
        telemetry.addData("X axis", orientation.thirdAngle);

        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        heading = AngleUnit.normalizeRadians(heading - offset);

        //If reset, set offset to the current ange
        //If we need to reset our zero angle, increment the offset with the current heading to do so
        if (gamepad1.x && !prevSlowState){
            offset += heading;
            gamepad1.rumble(0.0, 1.0, 300);
        }
        prevSlowState = gamepad1.x;

        double ly = Extensions.cubeInput(-gamepad1.left_stick_y, 0.2);
        double lx = Extensions.cubeInput(gamepad1.left_stick_x * 1.1, 0.2);
        double rx = Extensions.cubeInput(gamepad1.right_stick_x, 0.2);

        //rotate by the heading of the robot
        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
        lx = vector.getX();
        ly = vector.getY();

        double denom = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);

        leftFront.setPower((ly + lx + rx) / denom * powerMultiplier);
        leftBack.setPower((ly - lx + rx) / denom * powerMultiplier);
        rightFront.setPower((ly - lx - rx) / denom * powerMultiplier);
        rightBack.setPower((ly + lx - rx) / denom * powerMultiplier);

        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        telemetry.addData("hub 1", modules.get(0).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("hub 2", modules.get(1).getCurrent(CurrentUnit.AMPS));



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
