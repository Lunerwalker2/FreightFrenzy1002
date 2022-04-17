package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.vision.HubLevel;

public class MoveLiftToMidScoringPositionCommand extends ParallelCommandGroup {


    public MoveLiftToMidScoringPositionCommand(Lift lift, ScoringArm scoringArm, Bucket bucket) {
        addCommands(
                new LiftPositionCommand(lift, Lift.LiftPosition.MIDDLE),
                new InstantCommand(() -> {
                    scoringArm.setPosition(0.3);
                    scoringArm.loading = false;
                    bucket.close();
                })
        );
    }
}
