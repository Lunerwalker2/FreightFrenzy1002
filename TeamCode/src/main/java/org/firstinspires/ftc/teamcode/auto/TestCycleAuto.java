package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.commands.CarouselWheelCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "One Cycle with distance sensors")
public class TestCycleAuto extends AutoBase {


    private SampleMecanumDrive drive;
    private DistanceSensors distanceSensors;
    private CarouselWheel carouselWheel;

    private Trajectory goForward;
    private TrajectorySequence turnLeft;
    private Trajectory goToWarehouse;
    private TrajectorySequence backToHub;

    private Pose2d startPose = new Pose2d(6, 63, toRadians(-90));

    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        telemetry.addLine("Generating trajectories...");
        telemetry.update();

        goForward = drive.trajectoryBuilder(startPose)
                .forward(8)
                .build();

        turnLeft = drive.trajectorySequenceBuilder(goForward.end())
                .turn(toRadians(-90))
                .build();

        goToWarehouse = drive.trajectoryBuilder(turnLeft.end())
                .splineTo(new Vector2d(50, 60), toRadians(0))
                .build();

        backToHub = drive.trajectorySequenceBuilder(goToWarehouse.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 55), toRadians(180))
                .build();

        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

        distanceSensors = new DistanceSensors(hardwareMap);
        carouselWheel = new CarouselWheel(hardwareMap);

        schedule(
                new InstantCommand(() -> {
                        telemetry.addLine("The program started!");
                        telemetry.update();
                }),
                new WaitCommand(1000),
                new FollowTrajectoryCommand(drive, goForward, 0),
                new FollowTrajectorySequenceCommand(drive, turnLeft, 0),
                new FollowTrajectoryCommand(drive, goToWarehouse, 0),
                new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitCommand(300),
                                new RelocalizeCommand(
                                        (pose2d -> drive.setPoseEstimate(pose2d)),
                                        distanceSensors,
                                        () -> drive.getPoseEstimate().getHeading(),
                                        false
                                )
                        ),
                        new CarouselWheelCommand(carouselWheel, true, 2000)
                ),
                new FollowTrajectorySequenceCommand(drive, backToHub, 0)
        );
    }
}
