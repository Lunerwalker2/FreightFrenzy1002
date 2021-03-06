package org.firstinspires.ftc.teamcode.auto.cycle;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.DropFreight;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.DropPreloadFreight;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.RetractFromFreight;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.util.Extensions;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous
public class RedCycleAuto extends AutoBase {


    private SampleMecanumDrive drive;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private Lift lift;

    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;

    private Pose2d startPose = new Pose2d(8, -65, toRadians(180.0));


    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);


        leftIntake = new LeftIntake(hardwareMap);
        rightIntake = new RightIntake(hardwareMap);
        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        lift = new Lift(hardwareMap);
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, true, false);

        teamMarkerDetector.init();


        //commands here ig lol
        DropPreloadFreight dropPreloadFreight = new DropPreloadFreight(
                drive, lift, leftIntake, scoringArm, bucket, startPose, () -> hubLevel, true
        );

        DropFreight dropFreight = new DropFreight(
                drive, lift, rightIntake, scoringArm, bucket, true
        );

        RetractFromFreight retractFromFreight = new RetractFromFreight(
                drive, lift, rightIntake, scoringArm, bucket, true
        );


        //start vision
        teamMarkerDetector.startStream();
        while (!isStarted()){
            hubLevel = HubLevel.valueOf(teamMarkerDetector.getHubLevel().toString());
            telemetry.addLine("Ready For Start!");
            telemetry.addData("Hub Level", hubLevel);
            telemetry.update();
        }

        teamMarkerDetector.endStream();


        schedule(
                new SequentialCommandGroup(
                        //preload
                        new WaitCommand(500),
                        dropPreloadFreight,
                        new WaitCommand(100),
                        retractFromFreight,

                        //cycle 1
                        new WaitCommand(100),
                        new DropFreight(drive, lift, rightIntake, scoringArm, bucket, true),
                        new WaitCommand(100),
                        new RetractFromFreight(drive, lift, rightIntake, scoringArm, bucket, true),

                        //cycle 2
                        new WaitCommand(100),
                        new DropFreight(drive, lift, rightIntake, scoringArm, bucket, true),
                        new WaitCommand(100),
                        new RetractFromFreight(drive, lift, rightIntake, scoringArm, bucket, true),

                        //cycle 3
                        new WaitCommand(100),
                        new DropFreight(drive, lift, rightIntake, scoringArm, bucket, true),
                        new WaitCommand(100),
                        new RetractFromFreight(drive, lift, rightIntake, scoringArm, bucket, true),

                        new InstantCommand(() -> rightIntake.stop())
                )
        );

    }


    //Store the current heading for teleop
    @Override
    public void reset() {
        super.reset();
        Extensions.HEADING_SAVER = drive.getExternalHeading();
    }
}
