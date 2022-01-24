package org.firstinspires.ftc.teamcode.auto.warehouse;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.CrawlForwardUntilIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropFreightInHubCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPreLoadFreightCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.RetractAndGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.RetractFromPreLoadGoToWarehouseCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous
public class RedCycleAuto extends AutoBase {

    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private DistanceSensors distanceSensors;

    private DropPreLoadFreightCommand dropPreLoadFreightCommand;
    private RetractFromPreLoadGoToWarehouseCommand retractFromPreLoadGoToWarehouseCommand;
    private CrawlForwardUntilIntakeCommand crawlForwardUntilIntakeCommand1;
    private CrawlForwardUntilIntakeCommand crawlForwardUntilIntakeCommand2;
    private DropFreightInHubCommand dropFreightInHubCommand1;
    private DropFreightInHubCommand dropFreightInHubCommand2;
    private RetractAndGoToWarehouseCommand goToWarehouseCommand1;
    private RetractAndGoToWarehouseCommand goToWarehouseCommand2;
    private RelocalizeCommand relocalizeCommand1;
    private RelocalizeCommand relocalizeCommand2;

    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;


    private Pose2d startPose = new Pose2d(8.34375, -65.375, toRadians(180.0)); //left side aligned with left crease

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
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, true);

        teamMarkerDetector.init();

        telemetry.addLine("Scheduling Commands...");
        telemetry.update();

        dropPreLoadFreightCommand = new DropPreLoadFreightCommand(
                drive, lift, scoringArm, bucket, () -> hubLevel, true
        );

        retractFromPreLoadGoToWarehouseCommand = new RetractFromPreLoadGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, true, () -> hubLevel
        );

        dropFreightInHubCommand1 = new DropFreightInHubCommand(
                drive, lift, scoringArm, bucket, intake, true
        );

        dropFreightInHubCommand2 = new DropFreightInHubCommand(
                drive, lift, scoringArm, bucket, intake, true
        );

        goToWarehouseCommand1 = new RetractAndGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, true
        );

        goToWarehouseCommand2 = new RetractAndGoToWarehouseCommand(
                drive, lift, scoringArm, bucket, true
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
                crawlForwardUntilIntakeCommand1 = new CrawlForwardUntilIntakeCommand(
                        drive, intake, bucket, telemetry, true
                ),
                dropFreightInHubCommand1
//                goToWarehouseCommand1,
//                crawlForwardUntilIntakeCommand2,
//                dropFreightInHubCommand2,
//                goToWarehouseCommand2
                //TODO: DO NOT IMPLEMENT DISTANCE SENSORS UNTIL WIRE IS FIXED
        ));



    }


}
