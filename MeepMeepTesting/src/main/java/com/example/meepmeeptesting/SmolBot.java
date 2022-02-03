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


    public static double MAX_VEL = 35;
    public static double MAX_ACCEL = 35;
    public static double MAX_ANG_VEL = toRadians(200);
    public static double MAX_ANG_ACCEL = toRadians(200);
    public static double TRACK_WIDTH = 12.1;


    //length = 17.3125
    //width = 13.25


    private static final Pose2d blueStartingPosition = new Pose2d(8.34375, 65.375, toRadians(0));
    private static final Pose2d redStartingPosition =
            blueStartingPosition.copy(blueStartingPosition.getX(), -blueStartingPosition.getY(),
                    Angle.normDelta(blueStartingPosition.getHeading() + toRadians(180)));

    private static final Pose2d blueStartingPositionDuck = new Pose2d(-31.96875, 65.375, toRadians(0));
    private static final Pose2d redStartingPositionDuck =
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
//                                .splineToConstantHeading(new Vector2d(8, 50), toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(8, 45, toRadians(-30)), toRadians(-90))
                                .lineToLinearHeading(new Pose2d(10, 58, toRadians(-30)))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(0))
                                .splineToConstantHeading(new Vector2d(45, 64), toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("Intaking"))
                                .forward(7, getVelocityConstraint(5, toRadians(200), 12.1))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Freight Found, Relocalizing"))
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(13, 64.5), toRadians(180))
                                .splineToConstantHeading(new Vector2d(-10, 58), toRadians(-160))
                                .setReversed(false)
                                .build()
                );

        RoadRunnerBotEntity redCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(13, 18)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPosition)
                                .lineToConstantHeading(new Vector2d(-10, -58))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                                .splineToConstantHeading(new Vector2d(45, -64), toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("Intaking"))
                                .setReversed(false)
                                .setVelConstraint(getVelocityConstraint(5, toRadians(200), 12.1))
                                .back(5)
                                .forward(5)
                                .resetVelConstraint()
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Freight Found, Relocalizing"))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(180))
                                .splineToConstantHeading(new Vector2d(-10, -58), toRadians(160))
                                .setReversed(true)
                                .build()
                );

        RoadRunnerBotEntity blueDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(13, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStartingPositionDuck)
                                .lineToConstantHeading(new Vector2d(-10, 58))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .lineToConstantHeading(new Vector2d(-35, 50))
                                .lineToConstantHeading(new Vector2d(-50, 40))
                                .turn(toRadians(-30))
                                .lineToConstantHeading(new Vector2d(-57, 55))
                                .waitSeconds(3)
                                .forward(10)
                                .turn(toRadians(30))
                                .forward(20)
                                .strafeLeft(10)
                                .back(20,
                                        getVelocityConstraint(10, toRadians(180), 13)
                                )
                                .lineToConstantHeading(new Vector2d(-10, 58))
                                .waitSeconds(3)
                                .back(20)
                                .lineToLinearHeading(new Pose2d(-60, 40, toRadians(0)))
                                .build()
                );

        RoadRunnerBotEntity redDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(13, 18)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPositionDuck)
                                .lineToConstantHeading(new Vector2d(-10, -58))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .lineToConstantHeading(new Vector2d(-35, -50))
                                .lineToConstantHeading(new Vector2d(-50, -40))
                                .turn(toRadians(-150))
                                .lineToConstantHeading(new Vector2d(-57, -55))
                                .waitSeconds(3)
                                .forward(10)
                                .turn(toRadians(-30))
                                .forward(20)
                                .strafeRight(10)
                                .back(20,
                                        getVelocityConstraint(10, toRadians(180), 13)
                                )
                                .lineToConstantHeading(new Vector2d(-10, -58))
                                .waitSeconds(3)
                                .back(20)
                                .lineToLinearHeading(new Pose2d(-60, -40, toRadians(0)))
                                .build()
                );


        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(blueCycleRoute)
                .addEntity(redCycleRoute)
//                .addEntity(blueDuckRoute)
//                .addEntity(redDuckRoute)
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