package org.firstinspires.ftc.teamcode.commands.autocommands.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class GoToCarousel extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final IntakeSide intakeSide;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final CarouselWheel carouselWheel;
    private final boolean redSide;

    private TrajectorySequence trajectory;

    public GoToCarousel(SampleMecanumDrive drive, Lift lift, IntakeSide intakeSide,
                        ScoringArm scoringArm, Bucket bucket, CarouselWheel carouselWheel,
                        boolean redSide) {


        this.drive = drive;
        this.lift = lift;
        this.intakeSide = intakeSide;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.carouselWheel = carouselWheel;
        this.redSide = redSide;

    }


    @Override
    public void initialize() {
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                        .splineToConstantHeading(new Vector2d(40, -64), toRadians(0))
                        .splineToConstantHeading(new Vector2d(50, -64), toRadians(0))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(0))
                        .splineToConstantHeading(new Vector2d(40, 64), toRadians(0))
                        .splineToConstantHeading(new Vector2d(50, 64), toRadians(0))
                        .build();

        addCommands(
                new SequentialCommandGroup(
                        new FollowTrajectorySequenceCommand(drive, trajectory),
                        new InstantCommand(() -> {
                            carouselWheel.setWheelPower(0.5);
                        })
                        new WaitCommand()
                )
                ,
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new MoveLiftToLoadingPositionCommand(
                                lift, scoringArm, bucket
                        )
                ),
                );


        super.initialize();
    }
}
