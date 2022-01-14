package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MakeReadyToScoreCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.HubLevel;

public class DropPreLoadFreightCommand extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final HubLevel hubLevel;
    private final boolean redSide;

    private static final Pose2d blueStartingPosition =
            new Pose2d(8.34375, 65.375, toRadians(0.0));

    private static final Pose2d redStartingPosition =
            new Pose2d(8.34375, -65.375, toRadians(180.0));


    public DropPreLoadFreightCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm, Bucket bucket,
            HubLevel hubLevel, boolean redSide) {
        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.hubLevel = hubLevel;
        this.redSide = redSide;

        addRequirements(bucket, scoringArm);

        generateTrajectories();
    }

    @Override
    public void initialize() {
        addCommands(
                new FollowTrajectorySequenceCommand(drive, getPreLoadTrajectory()),
                new MoveLiftPositionCommand(lift,
                        (hubLevel == HubLevel.TOP) ? Lift.Positions.TOP :
                                (hubLevel == HubLevel.MIDDLE) ? Lift.Positions.MIDDLE :
                                        Lift.Positions.BOTTOM, 10),
                new SequentialCommandGroup(
                        new WaitCommand(600),
                        new InstantCommand(() -> {
                            switch (hubLevel) {
                                case TOP:
                                    scoringArm.scoringPosition();
                                    break;
                                case MIDDLE:
                                    scoringArm.setPosition(0.7);
                                    break;
                                case BOTTOM:
                                    scoringArm.setPosition(0.4);
                                    break;
                            }
                        }),
                        new WaitCommand(700),
                        new InstantCommand(bucket::dump)
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
                .lineTo(new Vector2d(-10, 60))
                .build();
        blueDriveToMiddleLevel = drive.trajectorySequenceBuilder(blueStartingPosition)
                .lineTo(new Vector2d(-10, 55))
                .build();
        blueDriveToBottomLevel = drive.trajectorySequenceBuilder(blueStartingPosition)
                .lineTo(new Vector2d(-10, 50))
                .build();
        redDriveToTopLevel = drive.trajectorySequenceBuilder(redStartingPosition)
                .lineTo(new Vector2d(-10, -60))
                .build();
        redDriveToMiddleLevel = drive.trajectorySequenceBuilder(redStartingPosition)
                .lineTo(new Vector2d(-10, -55))
                .build();
        redDriveToBottomLevel = drive.trajectorySequenceBuilder(redStartingPosition)
                .lineTo(new Vector2d(-10, -50))
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
