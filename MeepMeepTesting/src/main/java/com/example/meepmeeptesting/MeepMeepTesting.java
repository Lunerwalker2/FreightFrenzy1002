package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;

import static java.lang.Math.toRadians;

public class MeepMeepTesting {


    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = toRadians(160);
    public static double MAX_ANG_ACCEL = toRadians(160);
    public static double TRACK_WIDTH = 12;

    public static int HUB_LEVEL = (int) (Math.random() * 2);


    //large bot
    //blue side psoe of carousel wheel -62, 64
    //7.5 towards front, 9 to the rightx`

    private static final Pose2d blueStartingPosition = new Pose2d(6, 63.5, toRadians(90));
    private static final Pose2d redStartingPosition = blueStartingPosition.copy(blueStartingPosition.getX(), -blueStartingPosition.getY(), -blueStartingPosition.getHeading());

    public static void main(String[] args) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(600);


        RoadRunnerBotEntity blueTestCycle = new DefaultBotBuilder(mm)
                .setConstraints(30, 30, MAX_ANG_VEL, MAX_ANG_ACCEL, 15.6)
                .setDimensions(18, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.6, 64.0, toRadians(-90)))
                                .forward(15)
                                .turn(toRadians(90))
                                .splineTo(new Vector2d(40, 58), toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(10, 55), toRadians(180))
                                .build()
                );

        RoadRunnerBotEntity blueCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(12, 17)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStartingPosition)
                                .setReversed(true)
                                .splineTo(new Vector2d(-4, 38), toRadians(-115))
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(5, 57, toRadians(10)), toRadians(38))
                                .addDisplacementMarker(() -> System.out.println("marker"))
                                .splineToSplineHeading(new Pose2d(11, 62, toRadians(0)), toRadians(35))
                                .addDisplacementMarker(() -> System.out.println("marker"))
                                .splineToConstantHeading(new Vector2d(18, 63.5), toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("marker"))
//                                .splineTo(new Vector2d(20, 63.5), toRadians(2))
//                                .addDisplacementMarker(() -> System.out.println("marker"))
                                .splineToSplineHeading(new Pose2d(50, 63.5, toRadians(0)), toRadians(0))
                                //Intake
                                .waitSeconds(1.0)
                                .setReversed(true)
                                .splineTo(new Vector2d(16, 63.5), toRadians(180))
                                .splineToConstantHeading(new Vector2d(8, 60), toRadians(-120))
                                .splineToSplineHeading(new Pose2d(-4, 38, toRadians(60)), toRadians(-120))
                                .build()
                );

        RoadRunnerBotEntity redCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(12, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPosition)
                                .setReversed(true)
                                .splineTo(new Vector2d(-4, -38), toRadians(115))
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineTo(new Vector2d(16, -63), toRadians(-10))
                                .splineTo(new Vector2d(20, -63.5), toRadians(-3))
                                .splineTo(new Vector2d(50, -63.5), toRadians(0))
                                //Intake
                                .waitSeconds(1.0)
                                .setReversed(true)
                                .splineTo(new Vector2d(20, -63.5), toRadians(-180))
                                .splineTo(new Vector2d(-4, -40), toRadians(115))
                                .build()
                );

/*
        RoadRunnerBotEntity duckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(25.0, -63, toRadians(0)))
                                .forward(10)
                                .turn(toRadians(-90))
                                .lineToConstantHeading(new Vector2d(-53, 56.5))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-58.0, 35.0))
                                .build()
                );

        RoadRunnerBotEntity blueHubDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, 64, toRadians(-90.0)))
                                .lineToConstantHeading(new Vector2d(-30.0, 40.0))
                                .turn(toRadians(-135.0))
                                .back(6)
                                .waitSeconds(3)
                                .forward(8)
                                .turn(toRadians(-45))
                                .lineToLinearHeading(new Pose2d(-53, 56.5, toRadians(90)))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-58.0, 35.0))
                                .build()
                );

        RoadRunnerBotEntity redHubDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, -64, toRadians(90.0)))
                                .lineToConstantHeading(new Vector2d(-30.0, -40.0))
                                .turn(toRadians(135.0))
                                .back(6)
                                .waitSeconds(3)
                                .forward(8)
                                .turn(toRadians(45))
                                .lineToLinearHeading(new Pose2d(-53, -56.5, toRadians(-90)))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-58.0, -35.0))
                                .build()
                );
                */


        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueTestCycle)
                .addEntity(blueCycleRoute)
                .addEntity(redCycleRoute)
                .start();

    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}