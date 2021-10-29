package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous(name="Tank Parking Auto")
public class ParkingAuto extends LinearOpMode {



    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    TeamMarkerDetector detector = new TeamMarkerDetector(hardwareMap);

    HubLevel hubLevel = HubLevel.BOTTOM;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        detector.initialize();

        detector.startStream();
        while(!isStarted() && !isStopRequested()){
            hubLevel = detector.getTeamMarkerPipeline().getHubLevel();

            telemetry.addData("Hub Level", hubLevel);
            telemetry.update();
        }
        detector.endStream();


        if (isStopRequested()) return;

        sleep(1000);

        setMotorPowers(0.8, 0.8);
        sleep(3000);
        setMotorPowers(0,0);



    }

    void setMotorPowers(double leftSpeed, double rightSpeed){
        leftFront.setPower(leftSpeed);
        leftBack.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        rightBack.setPower(rightSpeed);
    }
}
