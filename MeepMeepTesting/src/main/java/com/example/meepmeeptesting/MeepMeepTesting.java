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


    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = toRadians(180);
    public static double MAX_ANG_ACCEL = toRadians(180);
    public static double TRACK_WIDTH = 14.7;

    public static int HUB_LEVEL = (int) (Math.random() * 2);


    //blue side psoe of carousel wheel -62, 64
    //7.5 towards front, 9 to the right

    public static void main(String[] args) {

        System.setProperty("sun.java2d.opengl", "true");
        System.out.println("\u2b1b / \u2b1c");

        MeepMeep mm = new MeepMeep(600);

        RoadRunnerBotEntity duckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, 64, toRadians(-90.0)))
                                .forward(10)
                                .turn(toRadians(-90))
                                .lineToConstantHeading(new Vector2d(-54.5, 55.0))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-58.0, 35.0))
                                .build()
                );

        RoadRunnerBotEntity blueHubDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, 64, toRadians(-90.0)))
                                .lineToConstantHeading(new Vector2d(-32.0, 42.0))
                                .turn(toRadians(45.0))
                                .forward(6)
                                .waitSeconds(3)
                                .back(8)
                                .turn(toRadians(135))
                                .lineToLinearHeading(new Pose2d(-54.5, 55.0, toRadians(90)))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-58.0, 35.0))
                                .build()
                );

        RoadRunnerBotEntity redHubDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 17)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33.6, -64, toRadians(90.0)))
                                .lineToConstantHeading(new Vector2d(-32.0, -42.0))
                                .turn(toRadians(-45.0))
                                .forward(6)
                                .waitSeconds(3)
                                .back(8)
                                .turn(toRadians(-135))
                                .lineToLinearHeading(new Pose2d(-54.5, -55.0, toRadians(-90)))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-58.0, -35.0))
                                .build()
                );

        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(duckRoute)
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