package org.firstinspires.ftc.teamcode.auto.duck;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.CarouselWheelCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MakeReadyToLoadCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.DropPreLoadFreightCommandDuck;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.ParkInStorageUnitDuck;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.RetractFromPreLoadCommandDuck;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous
public class RedDuckAuto extends AutoBase {


    private SampleMecanumDrive drive;
    private Bucket bucket;
    private Lift lift;
    private ScoringArm scoringArm;
    private Intake intake;
    private CarouselWheel carouselWheel;

    private TrajectorySequence backFromCarousel;
    private TrajectorySequence goToHub;

    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;

    private Pose2d startPose = new Pose2d(-31.96875, -65.375, toRadians(180));

    @Override
    public void initialize() {
        super.initialize();


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();


        intake = new Intake(hardwareMap);
        intake.setSide(false);
        lift = new Lift(hardwareMap, telemetry);
        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        carouselWheel = new CarouselWheel(hardwareMap);
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, true,  true);

        backFromCarousel = drive.trajectorySequenceBuilder(new Pose2d(-57, -55, toRadians(30)))
                .forward(10)
                .turn(toRadians(-30))
                .forward(20)
                .strafeRight(10)
                .back(20,
                        getVelocityConstraint(10, toRadians(180), 13),
                        getAccelerationConstraint(10)
                )
                .build();

        goToHub = drive.trajectorySequenceBuilder(backFromCarousel.end())
                .lineToConstantHeading(new Vector2d(-15, -58))
                .build();


        teamMarkerDetector.init();

        telemetry.addLine("Scheduling Commands...");
        telemetry.update();


        teamMarkerDetector.startStream();
        while (!isStarted()) {
            hubLevel = HubLevel.valueOf(teamMarkerDetector.getHubLevel().toString());
            telemetry.addLine("Ready For Start!");
            telemetry.addData("Hub Level", hubLevel);
            telemetry.update();
        }

        teamMarkerDetector.endStream();


        schedule(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            telemetry.addLine("The program started!");
                            telemetry.update();
                        }),
                        new WaitCommand(3000),
                        new DropPreLoadFreightCommandDuck(
                                drive, lift, scoringArm, bucket, () -> hubLevel, true
                        ).andThen(waitFor(700)),
                        new RetractFromPreLoadCommandDuck(
                                drive, lift, scoringArm, bucket, true, () -> hubLevel
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(3000),
                                new CarouselWheelCommand(
                                        carouselWheel, true
                                )
                        ),
                        new ParkInStorageUnitDuck(drive, true)
                )

        );
    }
}
