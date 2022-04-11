package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/*
Command that will keep a running average the position of the robot using the gyro and the distance sensors.

Command can be told to run for a set amount of time for now, and also can be interrupted.

We will use a supplier for the gyro angle in order to ensure that we can adapt to future
ways of obtaining heading (for example three-wheel odometry which doesn't use the IMU).
 */
public class RelocalizeCommand extends CommandBase {

    /*
     * If the robot is sitting in the exact middle of the field (0,0)
     * and facing straight down the x-axis (0 degrees), then the distances
     * from each sensor to the wall they are pointing at would be the following
     * in inches.
     */
    private static final double FORWARD_SENSOR_BASE_DISTANCE_TO_WALL = 66.78125;//63.125;
    private static final double BACKWARD_SENSOR_BASE_DISTANCE_TO_WALL = 64.75;
    private static final double LEFT_SENSOR_BASE_DISTANCE_TO_WALL = 69.425;
    private static final double RIGHT_SENSOR_BASE_DISTANCE_TO_WALL = 66.0625;


    /*
     * These are the relative positions of each sensor from the center of the
     * robot. They are relative because the actual coordinates doesn't matter,
     * only the horizontal and vertical distances (eg, the abs value of the x and y).
     *
     * y is forward, x is left/right
     * Inches
     */
    private static final Vector2d forwardSensorPosition = new Vector2d(-3.5, 5.53125);
    private static final Vector2d backwardSensorPosition = new Vector2d(-3.5, -8.59375);
    private static final Vector2d leftSensorPosition = new Vector2d(-7.1875, -1.15625);
    private static final Vector2d rightSensorPosition = new Vector2d(4, -4.78125);


    private final DistanceSensors distanceSensors;
    private final DoubleSupplier headingSupplier;
    private final Consumer<Pose2d> poseConsumer;
    private final boolean redSide;
    //Keep a timer so we can wait for the sensors to read.
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean done = false;

    /**
     * @param headingSupplier Supplier of the heading of the robot IN RADIANS.
     */
    public RelocalizeCommand(Consumer<Pose2d> poseConsumer, DistanceSensors distanceSensors, DoubleSupplier headingSupplier, boolean redSide) {
        super();
        this.distanceSensors = distanceSensors;
        this.headingSupplier = headingSupplier;
        this.poseConsumer = poseConsumer;
        this.redSide = redSide;
        addRequirements(distanceSensors);
    }

    /**
     * Returns if the current range readings are valid or not. cm
     */
    private static boolean isValidReadings(double front, double side) {
        return !(front < 6 ||
                front > 96 ||
                side > 100);
    }

    /*
    Happens repeatably until the command is no longer scheduled.
     */

    /**
     * Function that runs the trig needed to correctly offset the distance sensor measurements by
     * their horizontal and vertical positions on the robot.
     * <p>
     * The distance values given need to be the raw distance values from the sensors.
     * <p>
     * Heading is in pi to -pi range (left=+).
     * <p>
     * If left side is true, the left sensor will be used, and if false the right one.
     *
     * @param forwardDistance Forward distance sensor value, inches.
     * @param sideDistance    Side distance sensor value, inches
     * @param headingRad      Heading value, radians, euler
     * @param redSide         Side of the robot that the side sensor is on.
     * @return Array with (forward, side) absolute distances of the robot to the field walls.
     */
    private static double[] findRotatedDistance(double forwardDistance, double sideDistance, double headingRad, boolean redSide) {
        double[] newDistances = new double[2];


        //Rotate the vector with the sensor's position by the current heading
        //TODO: find exact error, the front sensor is slightly turned in its mount

        /*
        Red is the back of the robot facing the front wall, and the left side facing the side wall
        Blue is the front of the robot facing the front wall, and the right side facing the side wall

        For the red side, we need to rotate our heading by 180 since the robot is backwards.
        For the blue side, we don't, but the sensor is placed a little oddly in its case
         */
        Vector2d rotatedForwardSensorPosition = (redSide) ?
                backwardSensorPosition.rotated(AngleUnit.RADIANS.normalize(
                        headingRad + PI)) :
                forwardSensorPosition.rotated(AngleUnit.RADIANS.normalize(
                        headingRad - toRadians(4))
                );

        //Do the same for the side sensor
        Vector2d rotatedSideSensorPosition = (redSide) ?
                leftSensorPosition.rotated(AngleUnit.RADIANS.normalize(
                        headingRad + PI)) :
                rightSensorPosition.rotated(headingRad);

        /*
        Now find the theoretical distances from the walls, assuming no offset from one of the axes.

        This is just the reportedDistance * cos(heading) for both, since we know the
        hypot (reported distance) and the angle (heading of robot)
         */

        double correctedForwardDistance = forwardDistance * Math.cos(headingRad);
        double correctedSideDistance = sideDistance * Math.cos(headingRad);

        /*
        Finally, offset the distances by their y components. This is because we defined each sensor
        position in Q1, so the component needed is y for both of them.
         */

        //positive heading would cause the actual distance to be less, so add the offset
        newDistances[0] = correctedForwardDistance + rotatedForwardSensorPosition.getY();
        //positive heading would cause the actual distance to be more, so subtract the offset
        newDistances[1] = correctedSideDistance - rotatedSideSensorPosition.getY();

        return newDistances;
    }

    //This command will only run once

    /*
    Happens once when the command is scheduled.
     */
    @Override
    public void initialize() {
        super.initialize();
        //Start taking range measurements from the sensors
        timer.reset();
        done = false;
        distanceSensors.pingAll();
    }

    @Override
    public void execute() {

        if (timer.milliseconds() > 80 && !done) {
            //Find our current heading once so we don't have to keep reading it
            double heading = headingSupplier.getAsDouble();


            double forward = (!redSide) ?
                    distanceSensors.getForwardRange(DistanceUnit.INCH) :
                    distanceSensors.getBackwardRange(DistanceUnit.INCH);

            double side = distanceSensors.getLeftRange(DistanceUnit.INCH);

            //test for possible invalid values
//            if (!isValidReadings(forward, side)) return;

            //Find the rotated distances
            double[] rotatedDistances = findRotatedDistance(
                    forward,
                    side,
                    heading,
                    redSide
            );


            //Find our forward distance (x in field coordinates)
            double x = (!redSide) ?
                    (FORWARD_SENSOR_BASE_DISTANCE_TO_WALL - rotatedDistances[0]) :
                    (BACKWARD_SENSOR_BASE_DISTANCE_TO_WALL - rotatedDistances[0]);

            //Find our side distance (y in field coordinates)
            double y = (!redSide) ?
                    (RIGHT_SENSOR_BASE_DISTANCE_TO_WALL - rotatedDistances[1]) :
                    (rotatedDistances[1] - LEFT_SENSOR_BASE_DISTANCE_TO_WALL);

            //Update the user with the new position
            if (forward < 35 && forward > 10) poseConsumer.accept(new Pose2d(x, y, heading));
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return done;
    }


}
