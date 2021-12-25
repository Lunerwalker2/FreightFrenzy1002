package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.subsystems.Lift

class MoveLiftPositionCommand(private val lift: Lift, position: Lift.Positions ) : ProfiledPIDCommand(
        controller,
        lift::getLiftRawPosition,
        position.targetPosition.toDouble(),
        {output, _ -> lift.setLiftPower(output)},
        lift
) {

    companion object {

        private val controller = ProfiledPIDController(
                0.05, 0.0, 0.0,
                TrapezoidProfile.Constraints(
                        //TODO: Find this empirically
                        1000.0,
                        500.0
                )
        )
    }

    init {
        controller.setTolerance(20.0)
    }

    override fun isFinished(): Boolean {
        return controller.atGoal()
    }
}