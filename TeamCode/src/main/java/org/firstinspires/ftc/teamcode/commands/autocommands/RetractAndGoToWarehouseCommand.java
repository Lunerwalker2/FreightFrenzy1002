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

public class RetractAndGoToWarehouseCommand extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;

    public RetractAndGoToWarehouseCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm,
            Bucket bucket, boolean redSide
    ) {
        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.redSide = redSide;

        addCommands(
                new FollowTrajectorySequenceCommand(drive, getTrajectoryCommand()),
                new SequentialCommandGroup(
                        new WaitCommand(600),
                        new MakeReadyToLoadCommand(lift, scoringArm, bucket)
                )
        );

    }

    public TrajectorySequence getTrajectoryCommand() {
        return drive.trajectorySequenceBuilder(new Pose2d(10, (redSide) ? -60 : 60,
                (redSide) ? toRadians(180) : toRadians(0)))
                .setReversed(!redSide)
                .splineToConstantHeading(new Vector2d(15, (redSide) ? -64 : 64), toRadians(0))
                .splineToConstantHeading(new Vector2d(50, (redSide) ? -64 : 64), toRadians(0))
                .build();
    }
}
