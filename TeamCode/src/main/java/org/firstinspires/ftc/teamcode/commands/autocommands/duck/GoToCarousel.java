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
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class GoToCarousel extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final LeftIntake leftIntake;
    private final RightIntake rightIntake;
    private final CarouselWheel carouselWheel;
    private final boolean redSide;

    private TrajectorySequence trajectory;

    public GoToCarousel(SampleMecanumDrive drive, Lift lift, LeftIntake leftIntake, RightIntake rightIntake,
                        ScoringArm scoringArm, Bucket bucket, CarouselWheel carouselWheel,
                        boolean redSide) {


        this.drive = drive;
        this.lift = lift;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.carouselWheel = carouselWheel;
        this.redSide = redSide;

    }


    @Override
    public void initialize() {
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(-50, -40))
                        .turn(toRadians(-160))
                        .lineTo(new Vector2d(-63, -55))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(-50, 40))
                        .turn(toRadians(-86))
                        .lineTo(new Vector2d(-64, 59))
                        .build();

        addCommands(
                new InstantCommand(() -> {
                    leftIntake.intakePower(-0.4);
                    rightIntake.intakePower(-0.4);
                }),
                new SequentialCommandGroup(
                        new FollowTrajectorySequenceCommand(drive, trajectory),
                        new InstantCommand(() -> carouselWheel.setWheelPower(-0.6)),
                        new WaitCommand(3000),
                        new InstantCommand(() -> carouselWheel.setWheelPower(0))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new MoveLiftToLoadingPositionCommand(
                                lift, scoringArm, bucket
                        )
                ),
                new SequentialCommandGroup(
                        new WaitCommand(1400),
                        new InstantCommand(() -> {
                            leftIntake.intakePower(0);
                            rightIntake.intakePower(0);
                        })
                )

        );


        super.initialize();
    }
}
