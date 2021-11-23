package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensors;

import java.util.function.DoubleSupplier;

/*
Command that will keep a running average the position of the robot using the gyro and the distance sensors.

Command can be told to run for a set amount of time for now, and also can be interrupted.

We will use a supplier for the gyro angle in order to ensure that we can adapt to future
ways of obtaining heading (for example three-wheel odometry which doesn't use the IMU).
 */
public class RelocalizeCommand extends CommandBase {

    private static final double FORWARD_SENSOR_OFFSET = 65.0;
    private static final double LEFT_SENSOR_OFFSET = 65.0;
    private static final double RIGHT_SENSOR_OFFSET = 65.0;

    private final DistanceSensors distanceSensors;
    private final DoubleSupplier headingSupplier;
    private final boolean redSide;

    private Pose2d averagePosition = new Pose2d();

    /**
     *
     * @param headingSupplier Supplier of the heading of the robot IN RADIANS.
     */
    public RelocalizeCommand(DistanceSensors distanceSensors, DoubleSupplier headingSupplier, boolean redSide){
        super();
        this.distanceSensors = distanceSensors;
        this.headingSupplier = headingSupplier;
        this.redSide = redSide;
        addRequirements(distanceSensors);
    }

    @Override
    public void initialize(){
        //Start taking range measurements from the sensors
        distanceSensors.startReading();
    }

    @Override
    public void execute(){
        //Find our current position
        double heading = headingSupplier.getAsDouble();

        double x =
                distanceSensors.getForwardRange(DistanceUnit.INCH) * Math.cos(heading);

        double y = (redSide) ?
                        (distanceSensors.getRightRange(DistanceUnit.INCH)  * Math.cos(heading))-RIGHT_SENSOR_OFFSET :
                        (distanceSensors.getLeftRange(DistanceUnit.INCH)  * Math.cos(heading))-LEFT_SENSOR_OFFSET;

        Pose2d currentPosition = new Pose2d(x, y, heading);

        //Average the two
        averagePosition = new Pose2d(
                (averagePosition.getX() + currentPosition.getX()) / 2.0,
                (averagePosition.getY() + currentPosition.getY()) / 2.0,
                AngleUnit.normalizeRadians((averagePosition.getHeading() + currentPosition.getHeading()) / 2.0)
        );
    }

    @Override
    public void isFinished()


}
