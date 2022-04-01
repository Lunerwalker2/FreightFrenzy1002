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

    private DcMotorEx rightFront, leftFront, rightBack, leftBack;
    private BNO055IMU imu;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private Lift lift;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
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
        leftIntake = new LeftIntake(hardwareMap);
        rightIntake = new RightIntake(hardwareMap);

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


        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(scoringArm::scoringPosition, scoringArm::scoringPosition);

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

    }
}
