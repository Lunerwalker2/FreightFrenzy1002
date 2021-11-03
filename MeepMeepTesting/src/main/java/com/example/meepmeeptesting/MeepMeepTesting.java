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
    public static double MAX_ANG_VEL = toRadians(180);
    public static double MAX_ANG_ACCEL = toRadians(180);
    public static double TRACK_WIDTH = 14.7;

    public static int HUB_LEVEL = (int) (Math.random() * 2);


    public static void main(String[] args) {


        MeepMeep mm = new MeepMeep(600);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42.0, 54.5, toRadians(-90.0)))
                                .forward(10)
                                .turn(toRadians(-90))
                                .lineToConstantHeading(new Vector2d(-62.0, 44.5))

//                                .setReversed(true)
//                                .lineTo(new Vector2d(-56, 58))
//                                .addDisplacementMarker(() -> {
//                                    System.out.println("Carousel spinning!");
//                                })
//                                .waitSeconds(3)
//                                .setReversed(false)
//                                .setVelConstraint(getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH))
//                                .lineTo(new Vector2d(-10, 48))
//                                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
//                                    if (HUB_LEVEL == 0) System.out.println("Moving Arm to Level 1");
//                                    else if (HUB_LEVEL == 1)
//                                        System.out.println("Moving Arm to Level 2");
//                                    else System.out.println("Moving Arm to Level 3");
//                                })
//                                .resetVelConstraint()
//                                .turn(toRadians(-90))
//                                .forward(5)
//                                .addDisplacementMarker(() -> {
//                                    System.out.println("Dropping freight");
//                                })
//                                .waitSeconds(3)
//                                .back(8)
//                                .waitSeconds(2)
//                                .turn(toRadians(90))
//                                .splineTo(new Vector2d(12, 45), 0)
//                                .forward(40)

                                .build()
                );

        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
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