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

    private Pose2d startPose = new Pose2d(9.6, 64.0, toRadians(-90));

    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        telemetry.addLine("Generating trajectories...");
        telemetry.update();

        goForward = drive.trajectoryBuilder(startPose)
                .forward(15)
                .build();

        turnLeft = drive.trajectorySequenceBuilder(goForward.end())
                .turn(toRadians(-90))
                .build();

        goToWarehouse = drive.trajectoryBuilder(turnLeft.end())
                .splineTo(new Vector2d(40, 58), toRadians(0))
                .build();

        backToHub = drive.trajectorySequenceBuilder(goToWarehouse.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 55), toRadians(180))
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
                        new WaitCommand(1000),
                        new FollowTrajectoryCommand(drive, goForward),
                        new FollowTrajectorySequenceCommand(drive, turnLeft),
                        new FollowTrajectoryCommand(drive, goToWarehouse).andThen(waitFor(400)),
                        new ParallelDeadlineGroup(
                                new CarouselWheelCommand(carouselWheel, true).withTimeout(500),
                                new RelocalizeCommand(
                                        (pose) -> drive.setPoseEstimate(new Pose2d(
                                                pose.getX(),
                                                pose.getY(),
                                                pose.getHeading()
                                        )),
                                        distanceSensors,
                                        () -> drive.getExternalHeading(),
                                        false
                                )
                        ).andThen(waitFor(300)),
                        new FollowTrajectorySequenceCommand(drive, backToHub)
                )
        );
    }
}
