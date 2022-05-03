package org.firstinspires.ftc.teamcode.commands.autocommands.cycle;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.HubLevel;

import java.util.function.Supplier;

public class DropPreloadFreight extends ParallelCommandGroup {


    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final IntakeSide intakeSide;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;
    private final Supplier<HubLevel> getHubLevel;


    private Trajectory blueTop;
    private Trajectory blueMid;
    private Trajectory blueBottom;
    private Trajectory redTop;
    private Trajectory redMid;
    private Trajectory redBottom;


    public DropPreloadFreight(SampleMecanumDrive drive, Lift lift, IntakeSide intakeSide,
                              ScoringArm scoringArm, Bucket bucket, Pose2d startPose,
                              Supplier<HubLevel> getHubLevel, boolean redSide) {

        this.drive = drive;
        this.lift = lift;
        this.intakeSide = intakeSide;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.getHubLevel = getHubLevel;
        this.redSide = redSide;


        blueTop = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-9, 64))
                .build();
        blueMid = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-10, 58))
                .build();
        blueBottom = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, 44.5))
                .build();
        redTop = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-10, -60))
                .build();
        redMid = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-10, -57))
                .build();
        redBottom = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-12, -46))
                .build();

    }

    @Override
    public void initialize() {
        addCommands(
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new FollowTrajectoryCommand(drive, getPreLoadTrajectory(getHubLevel.get()))
                )
        );
        switch (getHubLevel.get()) {
            case TOP:


            case MIDDLE:
                addCommands(
                        new MoveLiftToScoringPositionCommand(lift, scoringArm, bucket, getHubLevel.get()),
                        new SequentialCommandGroup(
                                new WaitCommand(2300),
                                new InstantCommand(bucket::open)
                        )
                );
                break;
            case BOTTOM:
                addCommands(
                        new InstantCommand(() -> {
                            scoringArm.setPosition(0.225);
                            bucket.close();
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(2300),
                                new InstantCommand(() -> {
                                    scoringArm.setPosition(0.28);
                                    bucket.open();
                                })
                        )
                );

        }
        super.initialize();
    }


    private Trajectory getPreLoadTrajectory(HubLevel hubLevel) {
        Trajectory trajectory;
        switch (hubLevel) {
            case TOP:
                trajectory = (redSide) ? redTop : blueTop;
                break;
            case MIDDLE:
                trajectory = (redSide) ? redMid : blueMid;
                break;
            case BOTTOM:
                trajectory = (redSide) ? redBottom : blueBottom;
                break;
            default:
                trajectory = null;
        }
        return trajectory;
    }


}
