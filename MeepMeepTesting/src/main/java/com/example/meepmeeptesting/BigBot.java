package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

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
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class BigBot {


    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = toRadians(160);
    public static double MAX_ANG_ACCEL = toRadians(160);
    public static double TRACK_WIDTH = 15;

    public static int HUB_LEVEL = (int) (Math.random() * 2);


    //large bot
    //blue side psoe of carousel wheel -62, 64
    //7.5 towards front, 9 to the rightx`

    private static final Pose2d blueStartingPosition = new Pose2d(6, 63.5, toRadians(90));
    private static final Pose2d redStartingPosition = blueStartingPosition.copy(blueStartingPosition.getX(), -blueStartingPosition.getY(), -blueStartingPosition.getHeading());

    public static void main(String[] args) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(600);

        RoadRunnerBotEntity blueHubDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, 64, toRadians(90.0)))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-32.0, 40.0))
                                .turn(toRadians(45.0))
                                .setReversed(false)
                                .back(6)
                                .waitSeconds(3)
                                .forward(8)
                                .turn(toRadians(-45))
                                .lineToLinearHeading(new Pose2d(-62, 61, toRadians(90)))
                                .waitSeconds(2)
                                .back(14)
                                .turn(toRadians(-90))
                                .forward(110)
//                                .lineToConstantHeading(new Vector2d(-58.0, 35.0))
                                .build()
                );

        RoadRunnerBotEntity redHubDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, -64, toRadians(-90.0)))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-32.0, -40.0))
                                .turn(toRadians(-45.0))
                                .setReversed(false)
                                .back(6)
                                .waitSeconds(3)
                                .forward(8)
                                .turn(toRadians(45))
                                .lineToLinearHeading(new Pose2d(-62, -61, toRadians(-90)))
                                .waitSeconds(2)
                                .back(14)
                                .turn(toRadians(90))
                                .forward(110)
                                .build()
                );

/*
        RoadRunnerBotEntity blueTestCycle = new DefaultBotBuilder(mm)
                .setConstraints(30, 30, MAX_ANG_VEL, MAX_ANG_ACCEL, 15.6)
                .setDimensions(18, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, -64.0, toRadians(-90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-4.5, -40.0), Math.toRadians(112.0))
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(9.0, -62.0, Math.toRadians(-10.0)), Math.toRadians(-21.0))
                                .splineToSplineHeading(new Pose2d(19.0, -63.5, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .splineTo(new Vector2d(50.0, -63.5), Math.toRadians(0.0))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(19.0, -63.5, Math.toRadians(0.0)), Math.toRadians(180.0))
                                .splineToSplineHeading(new Pose2d(9.0, -62.0, Math.toRadians(-10.0)), Math.toRadians(159.0))
                                .splineToSplineHeading(new Pose2d(-4.5, -40.0, Math.toRadians(-75.0)), Math.toRadians(105.0))
                                .setReversed(false)
                                .build()
                );
                */



        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueTestCycle)
                .addEntity(blueHubDuckRoute)
                .addEntity(redHubDuckRoute)
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