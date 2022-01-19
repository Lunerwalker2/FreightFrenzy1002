package org.firstinspires.ftc.teamcode.auto.warehouse;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.autocommands.CrawlForwardUntilIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropFreightInHubCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPreLoadFreightCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.RetractAndGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.RetractFromPreLoadGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
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

    private DropPreLoadFreightCommand dropPreLoadFreightCommand;
    private RetractFromPreLoadGoToWarehouseCommand retractFromPreLoadGoToWarehouseCommand;
    private CrawlForwardUntilIntakeCommand crawlForwardUntilIntakeCommand;
    private DropFreightInHubCommand dropFreightInHubCommand;
    private RetractAndGoToWarehouseCommand goToWarehouseCommand;

    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.BOTTOM;


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
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, false);

        teamMarkerDetector.init();

        telemetry.addLine("Scheduling Commands...");
        telemetry.update();

        dropPreLoadFreightCommand = new DropPreLoadFreightCommand(
                drive, lift, scoringArm, bucket, () -> hubLevel, false
        );

        retractFromPreLoadGoToWarehouseCommand = new RetractFromPreLoadGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, false, () -> hubLevel
        );

        crawlForwardUntilIntakeCommand = new CrawlForwardUntilIntakeCommand(
                drive, intake, bucket, false
        );

        dropFreightInHubCommand = new DropFreightInHubCommand(
                drive, lift, scoringArm, bucket, intake, false
        );

        goToWarehouseCommand = new RetractAndGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, false
        );

        teamMarkerDetector.startStream();
        while (!isStarted()) {
            hubLevel = teamMarkerDetector.getTeamMarkerPipeline().getHubLevel();
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
                dropFreightInHubCommand,
                goToWarehouseCommand
//                crawlForwardUntilIntakeCommand
//                dropFreightInHubCommand.andThen(waitFor(500)),
//                goToWarehouseCommand.andThen(waitFor(500))

        ));


    }


}
