package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class LiftPositionCommand extends CommandBase {



    private PIDFController liftController;
    private PIDCoefficients coefficients = new PIDCoefficients(0.04, 0, 0);
    private double kStatic = 0.0;
    private double tolerance = 5;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;


    public LiftPositionCommand(Lift lift, Lift.LiftPosition position){
        this(lift, position.position);
    }

    public LiftPositionCommand(Lift lift, double targetPosition){
        this.lift = lift;
        this.targetPosition = targetPosition;

        liftController = new PIDFController(coefficients, 0, 0, kStatic);
        liftController.setOutputBounds(-1, 1);
    }

    @Override
    public void initialize(){
        //once
        lift.stop();
        liftController.reset();
        liftController.setTargetPosition(targetPosition);

    }



    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        //Update the lift power with the controller
        lift.setLiftPower(liftController.update(liftPosition));
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance

        return Math.abs(liftPosition - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted){
        lift.stop();
    }

}
