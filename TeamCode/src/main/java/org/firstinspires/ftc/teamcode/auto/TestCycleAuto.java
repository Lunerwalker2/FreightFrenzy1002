package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.commands.CarouselWheelCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Disabled
@Autonomous(name = "One Cycle with distance sensors")
public class TestCycleAuto extends AutoBase {


    private SampleMecanumDrive drive;
    private DistanceSensors distanceSensors;
    private CarouselWheel carouselWheel;

    private TrajectorySequence goToHubStart;
    private TrajectorySequence goToWarehouse;
    private TrajectorySequence goToWarehouseStart;
    private TrajectorySequence backToHub;

    private Pose2d startPose = new Pose2d(7.4, -64.0, toRadians(-90)); //left side aligned with left crease

    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        telemetry.addLine("Generating trajectories...");
        telemetry.update();

        goToHubStart = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-4.5, -40.0), toRadians(112.0))
                .waitSeconds(0.3)
                .setReversed(false)
                .build();

        goToWarehouseStart = drive.trajectorySequenceBuilder(goToHubStart.end())
                .splineToSplineHeading(new Pose2d(9.0, -62.0, toRadians(-10.0)), toRadians(-21.0))
                .splineToSplineHeading(new Pose2d(19.0, -63.5, toRadians(0.0)), toRadians(0.0))
                .splineTo(new Vector2d(40.0, -63.5), toRadians(0.0))
                .build();

        backToHub = drive.trajectorySequenceBuilder(new Pose2d(40.0, -63.5, toRadians(0.0)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(19.0, -63.5, toRadians(0.0)), toRadians(180.0))
                .splineToSplineHeading(new Pose2d(9.0, -62.0, toRadians(-10.0)), toRadians(159.0))
                .splineToSplineHeading(new Pose2d(-4.5, -40.0, toRadians(-75.0)), toRadians(105.0))
                .setReversed(false)
                .build();

        goToWarehouse = drive.trajectorySequenceBuilder(backToHub.end())
                .splineToSplineHeading(new Pose2d(9.0, -62.0, toRadians(-10.0)), toRadians(-21.0))
                .splineToSplineHeading(new Pose2d(19.0, -63.5, toRadians(0.0)), toRadians(0.0))
                .splineTo(new Vector2d(40.0, -63.5), toRadians(0.0))
                .build();


        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

        distanceSensors = new DistanceSensors(hardwareMap);
        carouselWheel = new CarouselWheel(hardwareMap);


        schedule(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            telemetry.addLine("The program started!");
                            telemetry.update();
                        }),
                        new FollowTrajectorySequenceCommand(drive, goToHubStart),
                        new FollowTrajectorySequenceCommand(drive, goToWarehouseStart),
                        new ParallelDeadlineGroup(
                                new CarouselWheelCommand(carouselWheel, true).withTimeout(500),
                                new RelocalizeCommand(
                                        (pose) -> drive.setPoseEstimate(new Pose2d(
                                                pose.getX(),
                                                pose.getY(),
                                                pose.getHeading()
                                        )),
                                        distanceSensors,
                                        drive::getExternalHeading,
                                        false
                                )
                        ).andThen(waitFor(300)),
                        new FollowTrajectorySequenceCommand(drive, backToHub),
                        new FollowTrajectorySequenceCommand(drive, goToWarehouse),
                        new ParallelDeadlineGroup(
                                new CarouselWheelCommand(carouselWheel, true).withTimeout(500),
                                new RelocalizeCommand(
                                        (pose) -> drive.setPoseEstimate(new Pose2d(
                                                pose.getX(),
                                                pose.getY(),
                                                pose.getHeading()
                                        )),
                                        distanceSensors,
                                        drive::getExternalHeading,
                                        false
                                )
                        ).andThen(waitFor(300)),
                        new FollowTrajectorySequenceCommand(drive, backToHub)
                )
        );

        telemetry.addLine("Ready for start!");
        telemetry.update();
    }
}
