package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

public class MoveLiftToScoringPositionCommand extends ParallelCommandGroup {


    public MoveLiftToScoringPositionCommand(Lift lift, ScoringArm scoringArm, Bucket bucket) {
        addCommands(
                new LiftPositionCommand(lift, Lift.LiftPosition.MIDDLE),
                new InstantCommand(() -> {
                    scoringArm.scoringPosition();
                    bucket.close();
                })
        );
    }
}
