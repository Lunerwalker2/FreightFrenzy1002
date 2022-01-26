package org.firstinspires.ftc.teamcode.auto.warehouse;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.CrawlForwardUntilIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.DropFreightInHubCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.DropPreLoadFreightCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.RetractAndGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.RetractFromPreLoadGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous
public class BlueCycleAuto extends AutoBase {

    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private DistanceSensors distanceSensors;

    private DropPreLoadFreightCommand dropPreLoadFreightCommand;
    private RetractFromPreLoadGoToWarehouseCommand retractFromPreLoadGoToWarehouseCommand;
    private DropFreightInHubCommand dropFreightInHubCommand1;
    private RetractAndGoToWarehouseCommand goToWarehouseCommand1;
    private RelocalizeCommand relocalizeCommand;

    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;


    private Pose2d startPose = new Pose2d(8.34375, 65.375, toRadians(0.0)); //left side aligned with left crease

    @Override
    public void initialize() {
        super.initialize();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        telemetry.addLine("Generating trajectories...");
        telemetry.update();


        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();


        intake = new Intake(hardwareMap);
        intake.setSide(false);
        lift = new Lift(hardwareMap, telemetry);
        scoringArm = new ScoringArm(hardwareMap);
        bucket = new Bucket(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, false, false);

        teamMarkerDetector.init();

        telemetry.addLine("Scheduling Commands...");
        telemetry.update();

        dropPreLoadFreightCommand = new DropPreLoadFreightCommand(
                drive, lift, scoringArm, bucket, () -> hubLevel, false
        );

        retractFromPreLoadGoToWarehouseCommand = new RetractFromPreLoadGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, false, () -> hubLevel
        );

        dropFreightInHubCommand1 = new DropFreightInHubCommand(
                drive, lift, scoringArm, bucket, intake, false
        );

        goToWarehouseCommand1 = new RetractAndGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, false
        );



        teamMarkerDetector.startStream();
        while (!isStarted()) {
            hubLevel = HubLevel.valueOf(teamMarkerDetector.getHubLevel().toString());
            telemetry.addLine("Ready For Start!");
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
                        dropPreLoadFreightCommand.andThen(waitFor(700)),
                        retractFromPreLoadGoToWarehouseCommand,
                        new CrawlForwardUntilIntakeCommand(
                                drive, intake, bucket, telemetry, false
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(100),
                                new RelocalizeCommand(
                                        drive::setPoseEstimate,
                                        distanceSensors,
                                        drive::getExternalHeading,
                                        false
                                )
                        ),
                        dropFreightInHubCommand1,
                        goToWarehouseCommand1,
                        new CrawlForwardUntilIntakeCommand(
                                drive, intake, bucket, telemetry, false
                        ),
                        new ParallelDeadlineGroup(
                                new WaitCommand(100),
                                new RelocalizeCommand(
                                        drive::setPoseEstimate,
                                        distanceSensors,
                                        drive::getExternalHeading,
                                        false
                                )
                        ),
                        new DropFreightInHubCommand(
                                drive, lift, scoringArm, bucket, intake, false
                        ),
                        new RetractAndGoToWarehouseCommand(
                                drive, lift, scoringArm, bucket, false
                        )
                )
        );


    }


}
