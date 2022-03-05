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
import org.firstinspires.ftc.teamcode.vision.HubLevel;

import java.util.function.Supplier;

public class RetractFromPreLoadGoToWarehouseCommand extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final Intake intake;
    private final boolean redSide;
    private final Supplier<HubLevel> getHubLevel;
    private HubLevel hubLevel;

    private static final Pose2d blueStartingPositionTop =
            new Pose2d(-10, 55, toRadians(0));
    private static final Pose2d blueStartingPositionMiddle =
            new Pose2d(-10, 50, toRadians(0));
    private static final Pose2d blueStartingPositionBottom =
            new Pose2d(-10, 49.5, toRadians(0));
    private static final Pose2d redStartingPositionTop =
            new Pose2d(-10, -54, toRadians(180));
    private static final Pose2d redStartingPositionMiddle =
            new Pose2d(-10, -50, toRadians(180));
    private static final Pose2d redStartingPositionBottom =
            new Pose2d(-10, -51, toRadians(180));



    public RetractFromPreLoadGoToWarehouseCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm,
            Bucket bucket, Intake intake, boolean redSide, Supplier<HubLevel> getHubLevel) {

        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.intake = intake;
        this.redSide = redSide;
        this.getHubLevel = getHubLevel;

        generateTrajectories();

    }

    @Override
    public void initialize() {

        hubLevel = getHubLevel.get();

        if(lift.getCurrentCommand() != null) lift.getCurrentCommand().cancel();

        addCommands(
                new FollowTrajectorySequenceCommand(drive, getTrajectoryCommand()),
                new SequentialCommandGroup(
                        new WaitCommand(800),
                        new MakeReadyToLoadCommand(lift, scoringArm, bucket, true)
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

    private static TrajectorySequence blueFromTopLevel;
    private static TrajectorySequence blueFromMiddleLevel;
    private static TrajectorySequence blueFromBottomLevel;
    private static TrajectorySequence redFromTopLevel;
    private static TrajectorySequence redFromMiddleLevel;
    private static TrajectorySequence redFromBottomLevel;

    public void generateTrajectories() {

        blueFromTopLevel = drive.trajectorySequenceBuilder(blueStartingPositionTop)
                .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 64), toRadians(0))
                .build();
        blueFromMiddleLevel = drive.trajectorySequenceBuilder(blueStartingPositionMiddle)
                .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(45, 64), toRadians(0))
                .build();
        blueFromBottomLevel = drive.trajectorySequenceBuilder(blueStartingPositionBottom)
                .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 64), toRadians(0))
                .build();
        redFromTopLevel = drive.trajectorySequenceBuilder(redStartingPositionTop)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -64), toRadians(0))
                .build();
        redFromMiddleLevel = drive.trajectorySequenceBuilder(redStartingPositionMiddle)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -64), toRadians(0))
                .build();
        redFromBottomLevel = drive.trajectorySequenceBuilder(redStartingPositionBottom)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -64), toRadians(0))
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
            default:
                return null;
        }
    }
}
