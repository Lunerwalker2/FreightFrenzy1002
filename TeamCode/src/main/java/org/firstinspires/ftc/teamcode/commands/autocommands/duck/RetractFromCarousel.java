package org.firstinspires.ftc.teamcode.commands.autocommands.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RetractFromCarousel extends ParallelCommandGroup {


    private SampleMecanumDrive drive;
    private IntakeSide intakeSide;
    private boolean redSide;

    private TrajectorySequence trajectory;

    public RetractFromCarousel(SampleMecanumDrive drive, IntakeSide intakeSide, boolean redSide) {

        this.drive = drive;
        this.intakeSide = intakeSide;
        this.redSide = redSide;
    }

    @Override
    public void initialize() {
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-60, -45, toRadians(-40)))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-60, 47, toRadians(40)))
                        .build();

        addCommands(
                new FollowTrajectorySequenceCommand(drive, trajectory),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new InstantCommand(() -> {
                            intakeSide.intakeDown();
                            intakeSide.intake();
                        })
                )
        );

        super.initialize();
    }


}
