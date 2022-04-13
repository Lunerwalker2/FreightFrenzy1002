package org.firstinspires.ftc.teamcode.commands.autocommands.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ParkInStorageUnit extends ParallelCommandGroup {



    private final SampleMecanumDrive drive;
    private final boolean redSide;

    private TrajectorySequence trajectory;

    public ParkInStorageUnit(SampleMecanumDrive drive, boolean redSide){

        this.redSide = redSide;
        this.drive = drive;
    }


    @Override
    public void initialize(){
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(-65, -60))
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(-65, 60))
                        .build();

        addCommands(
                new FollowTrajectorySequenceCommand(drive, trajectory)
        );

        super.initialize();
    }
}
