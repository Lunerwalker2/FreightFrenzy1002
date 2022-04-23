package org.firstinspires.ftc.teamcode.commands.autocommands.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DropDuck extends ParallelCommandGroup {


    private final SampleMecanumDrive drive;
    private final Lift lift;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;

    private TrajectorySequence trajectory;

    public DropDuck(SampleMecanumDrive drive, Lift lift, LeftIntake leftIntake, RightIntake rightIntake,
                    ScoringArm scoringArm, Bucket bucket, boolean redSide) {

        this.drive = drive;
        this.lift = lift;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.redSide = redSide;
    }

    @Override
    public void initialize() {

        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-40, -47, toRadians(135)))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-42, 48, toRadians(45)))
                        .build();

        addCommands(
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            leftIntake.intakePower(0.2);
                            leftIntake.intakeUp();
                        })
                ),
                new FollowTrajectorySequenceCommand(drive, trajectory),
                new SequentialCommandGroup(
                        new WaitCommand(1500),
                        new MoveLiftToScoringPositionCommand(lift, scoringArm, bucket),
                        new WaitCommand(300),
                        new InstantCommand(bucket::open),
                        new WaitCommand(1000)
                )
        );


        super.initialize();
    }
}
