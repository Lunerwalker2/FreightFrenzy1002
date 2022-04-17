package org.firstinspires.ftc.teamcode.commands.autocommands.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ScanForDuck extends ParallelCommandGroup {


    private SampleMecanumDrive drive;
    private IntakeSide intakeSide;
    private boolean redSide;

    private TrajectorySequence trajectory;

    public ScanForDuck(SampleMecanumDrive drive, IntakeSide intakeSide, boolean redSide) {

        this.drive = drive;
        this.intakeSide = intakeSide;
        this.redSide = redSide;
    }

    public void initialize() {
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-45, -45))
                        .lineToLinearHeading(new Pose2d(-45, -58.5, toRadians(-130)))
                        .lineToConstantHeading(new Vector2d(-57, -58.5))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-45, 45))
                        .lineToLinearHeading(new Pose2d(-45, 58.5, toRadians(130)))
                        .lineToConstantHeading(new Vector2d(-57, 58.5))
                        .build();

        addCommands(
                new SequentialCommandGroup(
                        new FollowTrajectorySequenceCommand(drive, trajectory),
                        new InstantCommand(() -> {
                            intakeSide.intakePower(0.2);
                            intakeSide.intakeUp();
                        })
                )
        );

        super.initialize();
    }

}
