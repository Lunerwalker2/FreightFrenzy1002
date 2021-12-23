package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

/**
 * Command that will raise the bucket, move the arm out, and raise the lift to a close position in readiness to score.
 *
 * Can be used in auto or teleop, but in teleop, be sure there is enough room and time to complete the command.
 *
 * Cancelling will stop the lift, however servos cannot really be stopped where they are, so they will continue moving.
 */
public class MakeReadyToScoreCommand extends ParallelCommandGroup {

    public MakeReadyToScoreCommand(Lift lift, ScoringArm scoringArm, Bucket bucket){
        addCommands(
                new MoveLiftPositionCommand(lift, Lift.Positions.BOTTOM),
                new InstantCommand(scoringArm::scoringPosition),
                new InstantCommand(bucket::down, bucket)

        );
    }

}
