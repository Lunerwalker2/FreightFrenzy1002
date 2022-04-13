package org.firstinspires.ftc.teamcode.auto.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.DropFreight;
import org.firstinspires.ftc.teamcode.commands.autocommands.cycle.RetractFromFreight;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.DropPreloadFreight;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.GoToCarousel;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.ParkInStorageUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.CarouselWheel;
import org.firstinspires.ftc.teamcode.subsystems.LeftIntake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.RightIntake;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.util.Extensions;
import org.firstinspires.ftc.teamcode.vision.HubLevel;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetector;

@Autonomous
public class RedDuckAuto extends AutoBase {


    private SampleMecanumDrive drive;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private Lift lift;
    private CarouselWheel carouselWheel;


    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;

    private Pose2d startPose = new Pose2d(-30.5, -65, toRadians(180.0));


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
        carouselWheel = new CarouselWheel(hardwareMap);
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, true, true);

        teamMarkerDetector.init();

        //commands here ig lol
        DropPreloadFreight dropPreloadFreight = new DropPreloadFreight(
                drive, lift, leftIntake, scoringArm, bucket, startPose, () -> hubLevel, true
        );

        GoToCarousel goToCarousel = new GoToCarousel(
                drive, lift, leftIntake, rightIntake, scoringArm, bucket, carouselWheel, true
        );

        ParkInStorageUnit parkInStorageUnit = new ParkInStorageUnit(
                drive, false
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
                        new WaitCommand(1000),
                        dropPreloadFreight,
                        new WaitCommand(100),
                        goToCarousel,
                        new WaitCommand(100),
                        parkInStorageUnit
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
