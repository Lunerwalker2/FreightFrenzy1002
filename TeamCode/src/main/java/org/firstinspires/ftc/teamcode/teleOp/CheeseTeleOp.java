package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

@TeleOp(name = "Main TeleOp")
public class CheeseTeleOp extends CommandOpMode {

    private DcMotorEx
            rightFront, leftFront, rightBack, leftBack, rightIntake, leftIntake, scoringArm;
    private BNO055IMU
            imu;
    private ScoringArm
            rightArm, leftArm;
    private final double offset = 0.0;

    GamepadEx controller1 = new GamepadEx(gamepad1);
    GamepadEx controller2 = new GamepadEx(gamepad2);

    // What will run at the start as well as the events
    @Override
    public void initialize() {
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf ");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        scoringArm = hardwareMap.get(DcMotorEx.class, "scoringArm");

        rightArm = new ScoringArm(hardwareMap);
        leftArm = new ScoringArm(hardwareMap);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

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
        double heading = orientation.secondAngle; //y axis

        double ly = -gamepad1.left_stick_y;
        double lx = Math.cos(heading) * gamepad1.left_stick_x - Math.sin(heading) * -gamepad1.left_stick_y;
        double rx = Math.sin(heading) * gamepad1.left_stick_x + Math.cos(heading) * -gamepad1.left_stick_y;

        leftFront.setPower(ly + lx + rx);
        leftBack.setPower(ly - lx + rx);
        rightFront.setPower(ly - lx - rx);
        rightBack.setPower(ly + lx - rx);

        if (gamepad1.right_trigger > 0) {
            rightIntake.setPower(1);
        }

        if (gamepad1.left_trigger > 0) {
            leftIntake.setPower(1);
        }

        if (-gamepad2.left_stick_y > 0) {
            scoringArm.setPower(0.5);
        }
        else if (-gamepad2.left_stick_y < 0) {
            scoringArm.setPower(-0.5);
        }

        if (gamepad2.right_bumper)

    }
}
