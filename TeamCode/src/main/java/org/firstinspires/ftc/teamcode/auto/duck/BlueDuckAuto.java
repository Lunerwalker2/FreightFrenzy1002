package org.firstinspires.ftc.teamcode.auto.duck;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.DropPreloadFreight;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.GoToCarousel;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.ParkInStorageUnit;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.RetractFromCarousel;
import org.firstinspires.ftc.teamcode.commands.autocommands.duck.ScanForDuck;
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
public class BlueDuckAuto extends AutoBase {


    private SampleMecanumDrive drive;
    private LeftIntake leftIntake;
    private RightIntake rightIntake;
    private ScoringArm scoringArm;
    private Bucket bucket;
    private Lift lift;
    private CarouselWheel carouselWheel;


    private TeamMarkerDetector teamMarkerDetector;
    private HubLevel hubLevel = HubLevel.TOP;

    private Pose2d startPose = new Pose2d(-30.5, 65, toRadians(0.0));


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
        teamMarkerDetector = new TeamMarkerDetector(hardwareMap, false, true);

        teamMarkerDetector.init();

        //commands here ig lol
        DropPreloadFreight dropPreloadFreight = new DropPreloadFreight(
                drive, lift, leftIntake, scoringArm, bucket, startPose, () -> hubLevel, false
        );

        GoToCarousel goToCarousel = new GoToCarousel(
                drive, lift, leftIntake, rightIntake, scoringArm, bucket, carouselWheel, false
        );

        ParkInStorageUnit parkInStorageUnit = new ParkInStorageUnit(
                drive, false
        );

        RetractFromCarousel retractFromCarousel = new RetractFromCarousel(
                drive, leftIntake, false
        );

        ScanForDuck scanForDuck = new ScanForDuck(
                drive, leftIntake, false
        );


        //start vision
        teamMarkerDetector.startStream();
        while (!isStarted()) {
            hubLevel = HubLevel.valueOf(teamMarkerDetector.getHubLevel().toString());
            telemetry.addLine("Ready For Start!");
            telemetry.addData("Hub Level", hubLevel);
            telemetry.update();
        }

        teamMarkerDetector.endStream();

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        dropPreloadFreight,
                        new WaitCommand(100),
                        goToCarousel,
                        new WaitCommand(100),
                        retractFromCarousel,
                        new WaitCommand(100),
                        scanForDuck
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
