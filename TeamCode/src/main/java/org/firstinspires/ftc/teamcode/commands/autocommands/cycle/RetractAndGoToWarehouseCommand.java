package org.firstinspires.ftc.teamcode.commands.autocommands.cycle;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MakeReadyToLoadCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RetractAndGoToWarehouseCommand extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final Intake intake;
    private final boolean redSide;

    private static final Pose2d blueStartingPosition =
            new Pose2d(-10, 53, toRadians(0));

    private static final Pose2d redStartingPosition =
            new Pose2d(-10, -56, toRadians(180));

    public RetractAndGoToWarehouseCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm,
            Bucket bucket, Intake intake, boolean redSide
    ) {
        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.intake = intake;
        this.redSide = redSide;

        generateTrajectory();

    }

    @Override
    public void initialize(){
        addCommands(
                new FollowTrajectorySequenceCommand(drive, getTrajectoryCommand()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> lift.setLiftPower(-1.0)),
                        new WaitCommand(100),
                        new InstantCommand(() -> lift.setLiftPower(0.0)),
                        new WaitCommand(600),
                        new MakeReadyToLoadCommand(lift, scoringArm, bucket, false)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(800),
                        new InstantCommand(() -> {
                            intake.setSide(redSide);
                            intake.intake();
                            intake.setFrontFlapUp();
                            intake.setFrontFlapUp();
                            intake.intakeFront();
                            intake.intakeBack();
                        })
                )
        );

        super.initialize();
    }

    private TrajectorySequence blueTrajectory;
    private TrajectorySequence redTrajectory;

    private void generateTrajectory() {
        blueTrajectory = drive.trajectorySequenceBuilder(blueStartingPosition)
                .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 64), toRadians(0))
                .build();

        redTrajectory = drive.trajectorySequenceBuilder(redStartingPosition)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -64), toRadians(0))
                .build();
    }

    public TrajectorySequence getTrajectoryCommand() {
        return (redSide) ? redTrajectory : blueTrajectory;
    }
}
