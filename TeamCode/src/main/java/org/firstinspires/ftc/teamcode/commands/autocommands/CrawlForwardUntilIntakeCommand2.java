package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class CrawlForwardUntilIntakeCommand2 extends ParallelDeadlineGroup {

    public CrawlForwardUntilIntakeCommand2(
            SampleMecanumDrive drive, Intake intake, Bucket bucket, Telemetry telemetry,
            boolean redSide) {

        super(
//                new CommandBase() {
//                    @Override
//                    public boolean isFinished(){
//                        telemetry.addData("Light detected", bucket.sensorRawLight());
//                        return bucket.isFreightDetected();
//                    }
//                },
                new WaitCommand(5000),
                new FollowTrajectorySequenceCommand(drive,
                        drive.trajectorySequenceBuilder(
                                new Pose2d(45, (redSide) ? -64 : 64,
                                        (redSide) ? toRadians(180) : toRadians(0))
                        )
                                .setReversed(redSide)
                                //Set it to go slow, even tho its really clunky
                                .setVelConstraint(
                                        SampleMecanumDrive.getVelocityConstraint(
                                                10, toRadians(180), DriveConstants.TRACK_WIDTH
                                        )
                                )
                                .forward(5)
                                .build()
                ),
                new SequentialCommandGroup(
                        new RunIntakeCommand(intake, redSide, true)
                )
        );

    }
}
