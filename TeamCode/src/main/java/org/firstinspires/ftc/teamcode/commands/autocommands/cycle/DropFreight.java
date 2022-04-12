package org.firstinspires.ftc.teamcode.commands.autocommands.cycle;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.HubLevel;

import java.util.function.Supplier;

public class DropFreight extends ParallelCommandGroup {


    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final IntakeSide intakeSide;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;

    private TrajectorySequence trajectory;


    public DropFreight(SampleMecanumDrive drive, Lift lift, IntakeSide intakeSide,
                       ScoringArm scoringArm, Bucket bucket, boolean redSide) {


        this.drive = drive;
        this.lift = lift;
        this.intakeSide = intakeSide;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.redSide = redSide;
    }

    @Override
    public void initialize() {
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(13.0, -64.5), Math.toRadians(180.0))
                        .splineToConstantHeading(new Vector2d(-7.0, -60), Math.toRadians(170))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(13.0, 64.5), Math.toRadians(180.0))
                        .splineToConstantHeading(new Vector2d(-7.0, 60.0), Math.toRadians(-170))
                        .build();

        //All of these happen in parallel
        addCommands(
                new FollowTrajectorySequenceCommand(drive, trajectory),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        //Raise arm and keep a constant power for the intake
                        new InstantCommand(() -> {
                            intakeSide.intakeUp();
                            intakeSide.intakePower(0.2);
                        }),
                        //Wait for the arm to raise and drop the freight into the bucket
                        new WaitCommand(1500),
                        //After a bit stop the intake, running it a little to make sure bucket isnt stuck
                        new InstantCommand(intakeSide::stop)
                ),
                //Wait a bit for the intake to load the freight
                new SequentialCommandGroup(
                        new WaitCommand(1400),
                        //Move lift and scoring out
                        new MoveLiftToScoringPositionCommand(
                                lift, scoringArm, bucket, HubLevel.TOP
                        ),
                        //Wait a bit to make sure things are stable
                        new WaitCommand(100),
                        //Open bucket and then wait a bit
                        new InstantCommand(bucket::open),
                        new WaitCommand(400)
                        //Retract lift in following command
                )

        );


        super.initialize();

    }
}
