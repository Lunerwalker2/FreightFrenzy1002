package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;

import static java.lang.Math.toRadians;

public class SmolBot {


    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = toRadians(200);
    public static double MAX_ANG_ACCEL = toRadians(200);
    public static double TRACK_WIDTH = 12.1;



    private static final Pose2d blueStartingPosition = new Pose2d(6, 63.5, toRadians(0));
    private static final Pose2d redStartingPosition =
            blueStartingPosition.copy(blueStartingPosition.getX(), -blueStartingPosition.getY(),
                    Angle.normDelta(blueStartingPosition.getHeading() + toRadians(180)));

    public static void main(String[] args) {

        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep mm = new MeepMeep(600);

        RoadRunnerBotEntity blueCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(13, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStartingPosition)
                                .lineToConstantHeading(new Vector2d(-10, 60))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .splineToConstantHeading(new Vector2d(15, 64), toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, 64), toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("Intaking"))
                                .forward(7, getVelocityConstraint(5, toRadians(200), 12.1))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Freight Found, Relocalizing"))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(15, 64), toRadians(180))
                                .splineToConstantHeading(new Vector2d(-10, 60), toRadians(-160))
                                .setReversed(false)
                                .build()
                );

        RoadRunnerBotEntity redCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(13, 18)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPosition)
                                .lineToConstantHeading(new Vector2d(-10, -60))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(15, -64), toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, -64), toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("Intaking"))
                                .back(7, getVelocityConstraint(5, toRadians(200), 12.1))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Freight Found, Relocalizing"))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(15, -64), toRadians(180))
                                .splineToConstantHeading(new Vector2d(-10, -60), toRadians(160))
                                .setReversed(true)
                                .build()
                );


        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
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