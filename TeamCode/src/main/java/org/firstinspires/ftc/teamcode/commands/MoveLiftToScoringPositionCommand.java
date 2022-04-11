package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.vision.HubLevel;

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

    public MoveLiftToScoringPositionCommand(Lift lift, ScoringArm scoringArm, Bucket bucket, HubLevel hubLevel){
        Lift.LiftPosition position = Lift.LiftPosition.TOP;
        switch (hubLevel) {
            case TOP:
                position = Lift.LiftPosition.TOP;
                break;
            case MIDDLE:
                position = Lift.LiftPosition.MIDDLE;
                break;
            case BOTTOM:
                position = Lift.LiftPosition.BOTTOM;
                break;
        }
        addCommands(
                new LiftPositionCommand(lift, position),
                new InstantCommand(() -> {
                    double armPosition = 0;
                    switch (hubLevel) {
                        case TOP:
                            armPosition = 0.6;
                            break;
                        case MIDDLE:
                            armPosition = 0.72;
                            break;
                        case BOTTOM:
                            armPosition = 0.9;
                            break;
                    }
                    scoringArm.setPosition(armPosition);
                    bucket.close();
                })
        );
    }
}
