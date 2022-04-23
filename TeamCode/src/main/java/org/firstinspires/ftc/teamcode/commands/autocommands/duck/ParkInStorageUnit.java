package org.firstinspires.ftc.teamcode.commands.autocommands.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ParkInStorageUnit extends ParallelCommandGroup {



    private final SampleMecanumDrive drive;
    private Lift lift;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
    private final boolean redSide;

    private TrajectorySequence trajectory;

    public ParkInStorageUnit(SampleMecanumDrive drive, Lift lift, ScoringArm scoringArm, Bucket bucket,
                             LeftIntake leftIntake, RightIntake rightIntake, boolean redSide){
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.redSide = redSide;
        this.drive = drive;
    }


    @Override
    public void initialize(){
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-65, -38, toRadians(90)))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-65, 40.1, toRadians(90)))
                        .build();

        addCommands(
                new FollowTrajectorySequenceCommand(drive, trajectory),
                new MoveLiftToLoadingPositionCommand(lift, scoringArm, bucket),
                new InstantCommand(() -> {
                    leftIntake.intakePower(-0.4);
                    rightIntake.intakePower(-0.4);
                    leftIntake.intakePosition(0.7);
                    rightIntake.intakePosition(0.4);
                })
        );

        super.initialize();
    }
}
