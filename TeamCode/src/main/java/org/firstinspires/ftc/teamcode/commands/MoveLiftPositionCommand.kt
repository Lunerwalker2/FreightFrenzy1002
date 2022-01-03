package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.subsystems.Lift

class MoveLiftPositionCommand(private val lift: Lift, position: Lift.Positions, tolerance: Double = 10.0) : ProfiledPIDCommand(
        controller,
        lift::getLiftRawPosition,
        position.targetPosition.toDouble(),
        {output, _ -> lift.setLiftPower(output)},
        lift
) {

    private var lastSetPoint = 0.0


    companion object {

        private val controller = ProfiledPIDController(
                0.05, 0.0, 0.0,
                TrapezoidProfile.Constraints(
                        //TODO: Find this empirically
                        2700.0,
                        2700.0
                )
        )
    }

    init {
        controller.setTolerance(tolerance)
    }

    override fun isFinished(): Boolean {
        return controller.atGoal()
    }
}