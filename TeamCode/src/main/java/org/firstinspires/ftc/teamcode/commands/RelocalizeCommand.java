package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

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
     *
     * //TODO: Find these
     */
    private static final double FORWARD_SENSOR_BASE_DISTANCE_TO_WALL = 65.0;
    private static final double LEFT_SENSOR_BASE_DISTANCE_TO_WALL = 65.0;
    private static final double RIGHT_SENSOR_BASE_DISTANCE_TO_WALL = 65.0;

    /*
     * These are the relative positions of each sensor from the center of the
     * robot. They are relative because the actual coordinates doesn't matter,
     * only the horizontal and vertical distances (eg, the abs value of the x and y).
     *
     * Inches
     */
    private static final Vector2d forwardSensorPosition = new Vector2d();
    private static final Vector2d leftSensorPosition = new Vector2d();
    private static final Vector2d rightSensorPosition = new Vector2d();


    private final DistanceSensors distanceSensors;
    private final DoubleSupplier headingSupplier;
    private final Consumer<Pose2d> poseConsumer;
    private final boolean leftSide;
    private boolean firstRun = true;

    private Pose2d averagePosition = new Pose2d();

    /**
     * @param headingSupplier Supplier of the heading of the robot IN RADIANS.
     */
    public RelocalizeCommand(Consumer<Pose2d> poseConsumer, DistanceSensors distanceSensors, DoubleSupplier headingSupplier, boolean leftSide) {
        super();
        this.distanceSensors = distanceSensors;
        this.headingSupplier = headingSupplier;
        this.poseConsumer = poseConsumer;
        this.leftSide = leftSide;
        addRequirements(distanceSensors);
    }

    /*
    Happens once when the command is scheduled.
     */
    @Override
    public void initialize() {
        //Start taking range measurements from the sensors
        distanceSensors.startReading();
    }

    /*
    Happens repeatably until the command is no longer scheduled.
     */

    @Override
    public void execute() {
        //Find our current heading once so we don't have to keep reading it
        double heading = headingSupplier.getAsDouble();

        //test for possible invalid values
        if (!isValidReadings(
                distanceSensors.getForwardRange(DistanceUnit.INCH),
                (leftSide) ?
                        distanceSensors.getLeftRange(DistanceUnit.INCH) :
                        distanceSensors.getRightRange(DistanceUnit.INCH))
        ) return;

        //Find the rotated distances
        double[] rotatedDistances = findRotatedDistance(
                distanceSensors.getForwardRange(DistanceUnit.INCH),
                (leftSide) ?
                        distanceSensors.getLeftRange(DistanceUnit.INCH) :
                        distanceSensors.getRightRange(DistanceUnit.INCH),
                heading,
                leftSide
        );


        //Find our forward distance (x in field coordinates)
        double x = (FORWARD_SENSOR_BASE_DISTANCE_TO_WALL - rotatedDistances[0]);

        //Find our side distance (y in field coordinates)
        double y = (leftSide) ?
                rotatedDistances[1] - RIGHT_SENSOR_BASE_DISTANCE_TO_WALL :
                LEFT_SENSOR_BASE_DISTANCE_TO_WALL - rotatedDistances[1];


        //Put them together in a position
        Pose2d currentPosition = new Pose2d(x, y, heading);

        //if its the first run we need to make sure we have an initial position for the average to work
        if (firstRun) {
            averagePosition = new Pose2d(currentPosition.getX(), currentPosition.getY(), currentPosition.getHeading());
            firstRun = false;
        } else {
            //Average the two
            averagePosition = new Pose2d(
                    (averagePosition.getX() + currentPosition.getX()) / 2.0,
                    (averagePosition.getY() + currentPosition.getY()) / 2.0,
                    AngleUnit.normalizeRadians((averagePosition.getHeading() + currentPosition.getHeading()) / 2.0)
            );
        }

        //Update the user with the new position
        poseConsumer.accept(averagePosition);
    }

    //Since this command will only be ended via cancellation, we dont need to specify an end condition

    @Override
    public void end(boolean interrupted) {
        distanceSensors.stopReading();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    /**
     * Returns if the current range readings are valid or not
     */
    private static boolean isValidReadings(double front, double side) {
        return (front < 8 ||
                front > 96 ||
                side < 3 ||
                side > 30);
    }

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
     * @param leftSide        Side of the robot that the side sensor is on.
     * @return Array with (forward, side) absolute distances of the robot to the field walls.
     */
    private static double[] findRotatedDistance(double forwardDistance, double sideDistance, double headingRad, boolean leftSide) {
        double[] newDistances = new double[2];


        //Rotate the vector with the sensor's position by the current heading
        Vector2d rotatedForwardSensorPosition = forwardSensorPosition.rotated(headingRad);
        //Do the same for the side sensor
        Vector2d rotatedSideSensorPosition = (leftSide) ?
                leftSensorPosition.rotated(headingRad) :
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


}
