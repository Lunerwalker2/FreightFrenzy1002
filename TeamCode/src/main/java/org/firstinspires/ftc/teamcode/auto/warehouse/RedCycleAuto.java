package org.firstinspires.ftc.teamcode.auto.warehouse;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.autocommands.CrawlForwardUntilIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropFreightInHubCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPreLoadFreightCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.RetractAndGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.RetractFromPreLoadAndCycleCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous
public class RedCycleAuto extends AutoBase {

    private SampleMecanumDrive drive;
    private DistanceSensors distanceSensors;
    private Intake intake;
    private Lift lift;
    private ScoringArm scoringArm;
    private Bucket bucket;

    private DropPreLoadFreightCommand dropPreLoadFreightCommand;
    private RetractFromPreLoadAndCycleCommand retractFromPreLoadAndCycleCommand;
    private DropFreightInHubCommand dropFreightInHubCommand;
    private RetractAndGoToWarehouseCommand goToWarehouseCommand;

    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;

    //TODO: Change
    private Pose2d startPose = new Pose2d(7.4, -64.0, toRadians(180)); //left side aligned with left crease

    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        telemetry.addLine("Generating trajectories...");
        telemetry.update();


        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

        distanceSensors = new DistanceSensors(hardwareMap, true);
        intake = new Intake(hardwareMap);
        intake.setSide(false);
        lift = new Lift(hardwareMap, telemetry);
        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap);

        teamMarkerDetector.init();

        telemetry.addLine("Scheduling Commands...");
        telemetry.update();

        dropPreLoadFreightCommand = new DropPreLoadFreightCommand(
                drive, lift, scoringArm, bucket, hubLevel, true
        );

        retractFromPreLoadAndCycleCommand = new RetractFromPreLoadAndCycleCommand(
                drive, lift, scoringArm, bucket, true, hubLevel
        );

        dropFreightInHubCommand = new DropFreightInHubCommand(
                drive, lift, scoringArm, bucket, intake, true
        );

        goToWarehouseCommand = new RetractAndGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, true
        );

        teamMarkerDetector.startStream();
        while (!isStarted()){
//            hubLevel = teamMarkerDetector.getTeamMarkerPipeline().getHubLevel();
            telemetry.addData("Hub Level", hubLevel);
            telemetry.update();
        }

        teamMarkerDetector.endStream();

        //Happens on start now

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("The program started!");
                    telemetry.update();
                }),
                dropPreLoadFreightCommand
//                retractFromPreLoadAndCycleCommand.andThen(waitFor(500)),
//                new CrawlForwardUntilIntakeCommand(
//                        drive, intake, bucket, true
//                )
//                dropFreightInHubCommand.andThen(waitFor(500)),
//                goToWarehouseCommand.andThen(waitFor(500))

        ));



    }



}
