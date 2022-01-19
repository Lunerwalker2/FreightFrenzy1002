package org.firstinspires.ftc.teamcode.commands.autocommands;

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
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RetractAndGoToWarehouseCommand extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;

    private static final Pose2d blueStartingPosition =
            new Pose2d(-10, -58, toRadians(180));

    private static final Pose2d redStartingPosition =
            new Pose2d(-10, 58, toRadians(0));

    public RetractAndGoToWarehouseCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm,
            Bucket bucket, boolean redSide
    ) {
        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.redSide = redSide;

        generateTrajectory();

    }

    @Override
    public void initialize(){
        addCommands(
                new FollowTrajectorySequenceCommand(drive, getTrajectoryCommand()),
                new SequentialCommandGroup(
                        new WaitCommand(800),
                        new MakeReadyToLoadCommand(lift, scoringArm, bucket, true)
                )
        );

        super.initialize();
    }

    private TrajectorySequence blueTrajectory;
    private TrajectorySequence redTrajectory;

    private void generateTrajectory() {
        blueTrajectory = drive.trajectorySequenceBuilder(blueStartingPosition)
                .splineToConstantHeading(new Vector2d(15, 64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(45, 64), toRadians(0))
                .build();

        redTrajectory = drive.trajectorySequenceBuilder(redStartingPosition)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(45, -64), toRadians(0))
                .build();
    }

    public TrajectorySequence getTrajectoryCommand() {
        return (redSide) ? redTrajectory : blueTrajectory;
    }
}
