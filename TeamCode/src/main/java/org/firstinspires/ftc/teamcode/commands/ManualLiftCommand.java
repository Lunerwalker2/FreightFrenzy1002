package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

public class ManualLiftCommand extends CommandBase {

    private final Lift lift;
    private final GamepadEx manipulator;

    public ManualLiftCommand(Lift lift, GamepadEx manipulator){

        addRequirements(lift);

        this.lift = lift;
        this.manipulator = manipulator;
    }

    @Override
    public void execute() {
        //Two dpad buttons cant be pressed at the same time so we don't have to worry about that.

        double multiplier = manipulator.getButton(GamepadKeys.Button.X) ? 0.5 : 1.0;

        //Check if the up button is pressed
        if (manipulator.getButton(GamepadKeys.Button.DPAD_UP) && !lift.atUpperLimit()) {
            lift.setLiftPower(0.8 * multiplier);
        }
        //Then check if the down is pressed
        else if (manipulator.getButton(GamepadKeys.Button.DPAD_DOWN) && !lift.atLowerLimit()) {
            lift.setLiftPower(-0.5 * multiplier);
        }
        //Otherwise, do nothing
        else {
            lift.stop();
        }
    }


}
