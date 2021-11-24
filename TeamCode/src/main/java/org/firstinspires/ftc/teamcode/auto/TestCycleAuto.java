package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Math.toRadians;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "One Cycle with distance sensors")
public class TestCycleAuto extends AutoBase {




    private SampleMecanumDrive drive;


    private Pose2d startPose = new Pose2d(0, 0, toRadians(0));

    @Override
    public void initialize(){
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);



        telemetry.addLine("Generating trajectories...");
        telemetry.update();


        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

    }
}
