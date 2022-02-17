package org.firstinspires.ftc.teamcode.commands.autocommands.cycle;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.HubLevel;

import java.util.function.Supplier;

public class DropPreLoadFreightCommand extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final Supplier<HubLevel> getHubLevel;
    private HubLevel hubLevel;
    private final boolean redSide;

    private static final Pose2d blueStartingPosition =
            new Pose2d(8.34375, 65.375, toRadians(0.0));

    private static final Pose2d redStartingPosition =
            new Pose2d(8.34375, -65.375, toRadians(180.0));


    public DropPreLoadFreightCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm, Bucket bucket,
            Supplier<HubLevel> getHubLevel, boolean redSide) {
        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.getHubLevel = getHubLevel;
        this.redSide = redSide;

        addRequirements(bucket, scoringArm);

        generateTrajectories();
    }

    @Override
    public void initialize() {
        hubLevel = getHubLevel.get();
        addCommands(
                new FollowTrajectorySequenceCommand(drive, getPreLoadTrajectory()),
                new SequentialCommandGroup(
                        new WaitCommand(1500),
                        new MoveLiftPositionCommand(lift,
                                (hubLevel == HubLevel.TOP) ? Lift.Positions.TOP :
                                        (hubLevel == HubLevel.MIDDLE) ? Lift.Positions.MIDDLE :
                                                Lift.Positions.BOTTOM, 5, 1800, 1700)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(2400),
                        new InstantCommand(() -> {
                            switch (hubLevel) {
                                case TOP:
                                    scoringArm.scoringPosition();
                                    break;
                                case MIDDLE:
                                    scoringArm.setPosition(0.45);
                                    break;
                                case BOTTOM:
                                    scoringArm.setPosition(0.43);
                                    break;
                            }
                        })
                ),
                new SequentialCommandGroup(
                        new WaitCommand(3500),
                        new InstantCommand(bucket::dump),
                        new WaitCommand(300)
                )
        );

        super.initialize();
    }

    private static TrajectorySequence blueDriveToTopLevel;
    private static TrajectorySequence blueDriveToMiddleLevel;
    private static TrajectorySequence blueDriveToBottomLevel;
    private static TrajectorySequence redDriveToTopLevel;
    private static TrajectorySequence redDriveToMiddleLevel;
    private static TrajectorySequence redDriveToBottomLevel;

    private void generateTrajectories() {
        blueDriveToTopLevel = drive.trajectorySequenceBuilder(blueStartingPosition)
                .lineTo(new Vector2d(-10, 53))
                .build();
        blueDriveToMiddleLevel = drive.trajectorySequenceBuilder(blueStartingPosition)
                .lineTo(new Vector2d(-10, 50))
                .build();
        blueDriveToBottomLevel = drive.trajectorySequenceBuilder(blueStartingPosition)
                .lineTo(new Vector2d(-10, 49.5))
                .build();
        redDriveToTopLevel = drive.trajectorySequenceBuilder(redStartingPosition)
                .lineTo(new Vector2d(-10, -56))
                .build();
        redDriveToMiddleLevel = drive.trajectorySequenceBuilder(redStartingPosition)
                .lineTo(new Vector2d(-10, -50))
                .build();
        redDriveToBottomLevel = drive.trajectorySequenceBuilder(redStartingPosition)
                .lineTo(new Vector2d(-10, -51))
                .build();
    }


    public TrajectorySequence getPreLoadTrajectory() {
        switch (hubLevel) {
            case TOP:
                return (redSide) ? redDriveToTopLevel : blueDriveToTopLevel;

            case MIDDLE:
                return (redSide) ? redDriveToMiddleLevel : blueDriveToMiddleLevel;

            case BOTTOM:
                return (redSide) ? redDriveToBottomLevel : blueDriveToBottomLevel;
            default:
                return null;
        }
    }

}
