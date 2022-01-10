package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;

import java.util.function.BooleanSupplier;

/**
 * This will be the default command for the lift in tele-op.
 * <p>
 * A default command in the FTCLib command framework is a command that is run whenever
 * no other commands using a particular subsystem are scheduled.
 * <p>
 * When another command is scheduled, this command is interrupted (the end() function).
 * When that command ends for any reason, and the subsystem isn't being used by another command,
 * this command is scheduled again.
 * <p>
 * "Scheduled" in this case means calling the initialize() function, then repeatedly calling the
 * execute() function as normal.
 */
public class ManualLiftCommand extends CommandBase {

    private final Lift lift;
    private final BooleanSupplier isScoringArmReady;
    private final BooleanSupplier isBucketReady;
    private final GamepadEx manipulator;

    public ManualLiftCommand(Lift lift, ScoringArm scoringArm, Bucket bucket,
                             GamepadEx manipulator) {
        addRequirements(lift); //Only add the lift since we aren't moving anything else

        this.lift = lift;
        isScoringArmReady = () -> scoringArm.getPosition() == ScoringArm.loadingPosition;
        isBucketReady = () -> !bucket.isDown();
        this.manipulator = manipulator;
    }

    @Override
    public void execute() {
        //Two dpad buttons cant be pressed at the same time so we don't have to worry about that.

        //Check if the up button is pressed
        if (manipulator.getButton(GamepadKeys.Button.DPAD_UP)) {
            //If it's at the top, then stop it
            if (!lift.atUpperLimit()) lift.setLiftPower(0.8);
            else lift.stopLift();
        }
        //Then check if the down is pressed
        else if (manipulator.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            //Check if its at the bottom or if it's near the bottom but the arm and bucket are still out
            if (!lift.atLowerLimit() &&
                    (lift.getLiftRawPosition() < 100 && !isScoringArmReady.getAsBoolean() &&
                            !isBucketReady.getAsBoolean())) {
                lift.setLiftPower(-0.6);
            } else lift.stopLift();
        }
        //Otherwise, do nothing
        else {
            lift.stopLift();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //If interrupted, just stop the lift
        lift.stopLift();
    }

}
