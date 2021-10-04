package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Simple Mecnaum Drive")
public class TestDriveTeleOp extends LinearOpMode {


    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;


    static final double VX_WEIGHT = 1;
    static final double VY_WEIGHT = 1;
    static final double OMEGA_WEIGHT = 1;

    double slowModeMult = 1.0;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");


        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }); //Sets the power decrease of RUE to 0%, making the max speed back to 100%

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()){
            if(isStopRequested()) return;

            Pose2d roadRunnerVoodooPowers = new Pose2d(cubeInput(-gamepad1.left_stick_y, 0.52), cubeInput(-gamepad1.left_stick_x,0.52), cubeInput(gamepad1.right_stick_x, 0.52));
            List<Double> drivePowers = getDrivePowers(normalizedVels(roadRunnerVoodooPowers));

            slowModeMult = gamepad1.left_bumper ? 0.5 : 1; //If the left bumper is pressed, reduce the speed


            lf.setPower(drivePowers.get(0) * slowModeMult);
            lb.setPower(drivePowers.get(1) * slowModeMult);
            rf.setPower(drivePowers.get(3) * slowModeMult);
            rb.setPower(drivePowers.get(2) * slowModeMult);

        }


    }

    double cubeInput(double input, double factor){
        double t = factor * Math.pow(input, 3);
        double r = input * (1-factor);
        return t+r;
    }

    List<Double> getDrivePowers(Pose2d drivePower){
        return MecanumKinematics.robotToWheelVelocities(drivePower,
                1.0, 1.0, 1.0);
    }

    Pose2d normalizedVels(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }
        return vel;
    }
}
