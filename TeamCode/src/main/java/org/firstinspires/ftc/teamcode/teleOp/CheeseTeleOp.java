package org.firstinspires.ftc.teamcode.teleOp;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToMidScoringPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CappingMech;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.util.Extensions;

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
    private CappingMech cappingMech;
    private double offset = 0.0;
    private double powerMultiplier = 1.0;
    private boolean prevSlowState = false;
    private boolean cappingMode = false;

    private MoveLiftToScoringPositionCommand moveLiftToScoringPositionCommand;
    private MoveLiftToMidScoringPositionCommand moveLiftToMidScoringPositionCommand;
    private MoveLiftToLoadingPositionCommand moveLiftToLoadingPositionCommand;
    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;


    // Starting items and continuous event running
    @Override
    public void initialize() {

        schedule(new BulkCacheCommand(hardwareMap));

        // Object declarations
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        leftIntake = new LeftIntake(hardwareMap);
        rightIntake = new RightIntake(hardwareMap);
        lift = new Lift(hardwareMap);
        cappingMech = new CappingMech(hardwareMap);
        carouselWheel = new CarouselWheel(hardwareMap);


        // Motors and Other Stuff
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf ");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Behaviors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);


        telemetry.addLine("Ready to start!");
        telemetry.update();

        // Commands
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

        // BEGIN THE FUN!!!
        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

        // Driver Triggers
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

        // Manipulator Triggers
        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                //If the trigger is pressed and the scoring arm is not in the robot then open the bucket
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

        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2)
                //Carousel wheel
                .whileActiveContinuous(() -> {
                    if (gamepad2.right_trigger < 0.8) carouselWheel.setWheelPower(0.6);
                    else carouselWheel.setWheelPower(0.8);
                })
                .whenInactive(() -> {
                    carouselWheel.setWheelPower(0.0);
                });

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


        // Driver Buttons
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(leftIntake::intakeDown, leftIntake::intakeUp);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(rightIntake::intakeDown, rightIntake::intakeUp);

        // Manipulator Buttons
        manipulator.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(() -> {
                            carouselWheel.setDirection(false);
                        },
                        () -> {
                            telemetry.addLine("Ready For Start!");

                            carouselWheel.setDirection(true);
                        });

        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(
                        () -> {
                            scoringArm.setPosition(0.2);
                            scoringArm.loading = false;
                            bucket.close();
                        },
                        () -> {
                            scoringArm.loadingPosition();
                            bucket.open();
                        }
                );

        manipulator.getGamepadButton(GamepadKeys.Button.Y)
                //Starts when pressed and ended when released
                .whenHeld(manualLiftResetCommand);


        manipulator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    cappingMode = !cappingMode;
                    ManualLiftCommand.cappingMode = cappingMode;
                });

    }

    // What will run continuously while running
    @Override
    public void run() {
        super.run();

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


        //Add the angle offset to be able to reset the 0 heading, and normalize it back to -pi to pi
        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - offset);

        //If reset, set offset to the current ange
        //If we need to reset our zero angle, increment the offset with the current heading to do so
        if (gamepad1.x && !prevSlowState) {
            offset += heading;
            gamepad1.rumble(0.0, 1.0, 300);
        }
        prevSlowState = gamepad1.x; // "Inefficient toggle" - Ryan 2022

        if (gamepad2.dpad_up && cappingMode) cappingMech.retract();
        else if (gamepad2.dpad_down && cappingMode) cappingMech.extend();
        else cappingMech.stop();


        if (gamepad2.dpad_left && cappingMode) cappingMech.incrementPosition();
        else if (gamepad2.dpad_right && cappingMode) cappingMech.decrementPosition();

        double ly = Extensions.cubeInput(-gamepad1.left_stick_y, 0.2);
        double lx = Extensions.cubeInput(gamepad1.left_stick_x * 1.1, 0.2);
        double rx = Extensions.cubeInput(gamepad1.right_stick_x, 0.2);

        // Rotate by the heading of the robot
        Vector2d vector = new Vector2d(lx, ly).rotated(-heading);
        lx = vector.getX();
        ly = vector.getY();

        double normalize = Math.max(abs(ly) + abs(lx) + abs(rx), 1.0);

        leftFront.setPower((ly + lx + rx) / normalize * powerMultiplier);
        leftBack.setPower((ly - lx + rx) / normalize * powerMultiplier);
        rightFront.setPower((ly - lx - rx) / normalize * powerMultiplier);
        rightBack.setPower((ly + lx - rx) / normalize * powerMultiplier);


        telemetry.update();
    }
}
