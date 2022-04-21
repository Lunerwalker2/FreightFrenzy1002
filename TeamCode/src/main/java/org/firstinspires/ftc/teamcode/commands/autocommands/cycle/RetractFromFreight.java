package org.firstinspires.ftc.teamcode.commands.autocommands.cycle;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.MoveLiftToLoadingPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RetractFromFreight extends ParallelCommandGroup {

    private final SampleMecanumDrive drive;
    private final Lift lift;
    private final IntakeSide intakeSide;
    private final ScoringArm scoringArm;
    private final Bucket bucket;
    private final boolean redSide;

    private TrajectorySequence trajectory;

    private static int cycleNum = 0;
    private double distanceAdd = 2;

//    private final Pose2d redStartingPosition =
//            new Pose2d(-10, -60, toRadians(180));
//    private final Pose2d blueStartingPosition =
//            new Pose2d(-10, 60, toRadians(0));


    public RetractFromFreight(SampleMecanumDrive drive, Lift lift, IntakeSide intakeSide,
                       ScoringArm scoringArm, Bucket bucket, boolean redSide) {


        this.drive = drive;
        this.lift = lift;
        this.intakeSide = intakeSide;
        this.scoringArm = scoringArm;
        this.bucket = bucket;
        this.redSide = redSide;

    }

    @Override
    public void initialize(){
        //Make the trajectory for the red or blue side
        /*
        A little funky, but instead of using the correct starting position for each preload
        if this is retracting from the preload (since the robot will be in different places for each level),
        we instead just ignore where we should be, and use the current position of the drive base.

        Technically it's a little slower and it means we aren't really correcting for any drift,
        but since its very soon after start there likely isn't much drift anyways, and it lets us not write
        an entirely separate command for retracting from the preload.
         */
//        trajectory = (redSide) ?
//                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .setReversed(true)
//                        .splineToConstantHeading(new Vector2d(13, -64), toRadians(0))
//                        .splineToConstantHeading(new Vector2d(32, -65), toRadians(0))
//                        .splineToConstantHeading(new Vector2d(37, -65), toRadians(0))
//                        .build() :
//                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .splineToConstantHeading(new Vector2d(13, 64), toRadians(0))
//                        .splineToConstantHeading(new Vector2d(32, 65), toRadians(0))
//                        .splineToConstantHeading(new Vector2d(37, 65), toRadians(0))
//                        .build();
        trajectory = (redSide) ?
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(10, -65, toRadians(180)))
                        .splineToConstantHeading(new Vector2d(40 + (cycleNum * distanceAdd), -65), toRadians(0))
                        .strafeRight(22)
                        .build() :
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(10, 65, toRadians(0)))
                        .splineToConstantHeading(new Vector2d(36 + (cycleNum * distanceAdd), 65), toRadians(0))
                        .build();

        clearGroupedCommands();
        addCommands(
                //Drive to the warehouse
                new FollowTrajectorySequenceCommand(drive, trajectory),
                //Retract the lift
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new MoveLiftToLoadingPositionCommand(
                                lift, scoringArm, bucket
                        )
                ),
                //Drop the intake
                new InstantCommand(() -> {
                    intakeSide.intakeDown();
                    intakeSide.intake();
                })
//                new WaitUntilCommand(intakeSide::freightDetected)
        );

        cycleNum++;

        super.initialize();
    }
}