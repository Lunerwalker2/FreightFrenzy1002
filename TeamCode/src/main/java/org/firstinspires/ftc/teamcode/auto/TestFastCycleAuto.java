package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CarouselWheelCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Red test cycle")
public class TestFastCycleAuto extends AutoBase {


    private SampleMecanumDrive drive;


    private TrajectorySequence goToHubStart;
    private TrajectorySequence goToWarehouse;
    private TrajectorySequence backToHub;

    private Pose2d startPose = new Pose2d(9.6, -64.0, toRadians(90));

    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        telemetry.addLine("Generating trajectories...");
        telemetry.update();

        goToHubStart = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-4.5, -40), toRadians(112))
                .waitSeconds(0.3)
                .setReversed(false)
                .build();

        goToWarehouse = drive.trajectorySequenceBuilder(goToHubStart.end())
                .splineToSplineHeading(new Pose2d(9, -62, toRadians(-10)), toRadians(-21))
                .splineToSplineHeading(new Pose2d(19, -63.5, toRadians(0)), toRadians(0))
                .splineTo(new Vector2d(50, -63.5), toRadians(0))
                .build();

        backToHub = drive.trajectorySequenceBuilder(goToWarehouse.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(19, -63.5, toRadians(0)), toRadians(180))
                .splineToSplineHeading(new Pose2d(9, -62, toRadians(-10)), toRadians(159))
                .splineToSplineHeading(new Pose2d(-4.5, -40, toRadians(-75)), toRadians(105))
                .setReversed(false)
                .build();

        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();


        schedule(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            telemetry.addLine("The program started!");
                            telemetry.update();
                        }),
                        new WaitCommand(1000),
                        new FollowTrajectorySequenceCommand(drive, goToHubStart).andThen(waitFor(300)),
                        new FollowTrajectorySequenceCommand(drive, goToWarehouse).andThen(waitFor(500)),
                        new FollowTrajectorySequenceCommand(drive, backToHub)
                )
        );
    }
}
