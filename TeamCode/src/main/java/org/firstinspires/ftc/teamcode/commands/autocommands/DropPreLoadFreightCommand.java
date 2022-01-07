package org.firstinspires.ftc.teamcode.commands.autocommands;

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
    private final Pose2d startPose;


    public DropPreLoadFreightCommand(
            SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm, Bucket bucket,
            HubLevel hubLevel, boolean redSide, Pose2d startPose) {
        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.hubLevel = hubLevel;
        this.redSide = redSide;
        this.startPose = startPose;

        addRequirements(lift, scoringArm, bucket);

        addCommands(
                new FollowTrajectoryCommand(drive, getPreLoadTrajectory())
                        .andThen(new InstantCommand(bucket::dump)),
                new MoveLiftPositionCommand(lift,
                        (hubLevel == HubLevel.TOP) ? Lift.Positions.TOP :
                                (hubLevel == HubLevel.MIDDLE) ? Lift.Positions.MIDDLE :
                                        Lift.Positions.BOTTOM, 10),
                new SequentialCommandGroup(
                        new WaitCommand(400),
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
                        })
                )


        );
    }


    public Trajectory getPreLoadTrajectory() {
        switch (hubLevel) {
            case TOP:
                return drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-10, (redSide) ? -60 : 60))
                        .build();

            case MIDDLE:
                return drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-10, (redSide) ? -55 : 55))
                        .build();

            case BOTTOM:
                return drive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(-10, (redSide) ? -50 : 50))
                        .build();
            default:
                return null;
        }
    }

}
