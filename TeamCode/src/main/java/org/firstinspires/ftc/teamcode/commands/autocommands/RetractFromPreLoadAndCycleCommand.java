package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.vision.HubLevel;

public class RetractFromPreLoadAndCycleCommand extends ParallelCommandGroup {


    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;
    private final HubLevel hubLevel;

    private static final Pose2d blueStartingPositionTop =
            new Pose2d(-10, 60, toRadians(0));
    private static final Pose2d blueStartingPositionMiddle =
            new Pose2d(-10, 55, toRadians(0));
    private static final Pose2d blueStartingPositionBottom =
            new Pose2d(-10, 50, toRadians(0));
    private static final Pose2d redStartingPositionTop =
            new Pose2d(-10, -60, toRadians(180));
    private static final Pose2d redStartingPositionMiddle =
            new Pose2d(-10, -55, toRadians(180));
    private static final Pose2d redStartingPositionBottom =
            new Pose2d(-10, -50, toRadians(180));



    public RetractFromPreLoadAndCycleCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm,
            Bucket bucket, boolean redSide, HubLevel hubLevel
    ) {

        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.redSide = redSide;
        this.hubLevel = hubLevel;

        generateTrajectories();

    }

    @Override
    public void initialize() {
        addCommands(
                new FollowTrajectorySequenceCommand(drive, getTrajectoryCommand()),
                new SequentialCommandGroup(
                        new WaitCommand(600),
                        new MakeReadyToLoadCommand(lift, scoringArm, bucket)
                )
        );
    }

    private static TrajectorySequence blueFromTopLevel;
    private static TrajectorySequence blueFromMiddleLevel;
    private static TrajectorySequence blueFromBottomLevel;
    private static TrajectorySequence redFromTopLevel;
    private static TrajectorySequence redFromMiddleLevel;
    private static TrajectorySequence redFromBottomLevel;

    public void generateTrajectories() {
        blueFromTopLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .splineToConstantHeading(new Vector2d(15, 64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, 64), toRadians(0))
                .build();
        blueFromMiddleLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .splineToConstantHeading(new Vector2d(15, 64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, 64), toRadians(0))
                .build();
        blueFromBottomLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .splineToConstantHeading(new Vector2d(15, 64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, 64), toRadians(0))
                .build();
        redFromTopLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -64), toRadians(0))
                .build();
        redFromMiddleLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -64), toRadians(0))
                .build();
        redFromBottomLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -64), toRadians(0))
                .build();
    }



    public TrajectorySequence getTrajectoryCommand() {
        switch (hubLevel) {
            case TOP:
                return (redSide) ? redFromTopLevel : blueFromTopLevel;
            case MIDDLE:
                return (redSide) ? redFromMiddleLevel : blueFromMiddleLevel;
            case BOTTOM:
                return (redSide) ? redFromBottomLevel : blueFromBottomLevel;
            default: return null;
        }
    }
}
