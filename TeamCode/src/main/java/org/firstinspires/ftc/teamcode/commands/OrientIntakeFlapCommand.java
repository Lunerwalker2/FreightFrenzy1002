package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.interfaces.IntakeSide;

public class OrientIntakeFlapCommand extends CommandBase {



    private final IntakeSide intake;
    private final double kP = 0.1;
    private final double tolerance = 4;
    private final double ticksPerHalfRot = 28*5.2/2;
    private final double target = 0;
    private double error = 0;


    public OrientIntakeFlapCommand(IntakeSide intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        error = getError();
    }

    @Override
    public void execute(){
        error = getError();

        double output = kP * error;

        intake.intakePower(output);
    }

    @Override
    public boolean isFinished(){
        return error < tolerance;
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }

    private double getError(){
        return target - (intake.currentPosition() % ticksPerHalfRot);
    }
}
