package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class RunIntakeCommand extends CommandBase {

    private final Intake intake;
    private final boolean front;
    private final boolean moveInwards;

    /**
     * Command to run one of the intakes either forward or backwards.
     * @param intake The intake subsystem
     * @param front Whether to move the front or the back intake
     * @param moveInwards Whether to move inwards (intake) or outwards (outtake)
     */
    public RunIntakeCommand(Intake intake, boolean front, boolean moveInwards){
        this.intake = intake;
        this.front = front;
        this.moveInwards = moveInwards;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setSide(front);
        intake.intake();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }


}
