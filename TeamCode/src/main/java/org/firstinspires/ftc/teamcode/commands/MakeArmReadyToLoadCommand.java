package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

/**
 * Command that will move the arm in and raise claw for loading position
 *
 * Can be used in auto or teleop, but in teleop, be sure there is enough room and time to complete the command.
 *
 * Cancelling will stop the lift, however servos cannot really be stopped where they are, so they will continue moving.
 */
public class MakeArmReadyToLoadCommand extends ParallelCommandGroup {


    public MakeArmReadyToLoadCommand(Lift lift, ScoringArm scoringArm, Bucket bucket){
        addCommands(
                new InstantCommand(scoringArm::loadingPosition, scoringArm),
                new InstantCommand(bucket::load, bucket)
        );
    }
}
