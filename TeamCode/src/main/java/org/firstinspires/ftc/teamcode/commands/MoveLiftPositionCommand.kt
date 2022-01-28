package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ProfiledPIDCommand
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.subsystems.Lift


class MoveLiftPositionCommand(private val lift: Lift,
                              position: Lift.Positions,
                              tolerance: Double = 10.0,
                              maxVel: Double,
                              maxAccel: Double
) : ProfiledPIDCommand(
        ProfiledPIDController(
                0.04, 0.0, 0.0,
                TrapezoidProfile.Constraints(
                        //TODO: Find this empirically
                        maxVel,
                        maxAccel
                )
        ),
        lift::getLiftRawPosition,
        position.targetPosition.toDouble(),
        { output, _ -> lift.setLiftPower(output) },
        lift
) {

    constructor(lift: Lift,
                position: Lift.Positions,
                tolerance: Double = 10.0,
                ) : this(lift, position, tolerance, 2300.0, 2300.0)


    init {
        controller.setTolerance(tolerance)
    }

    override fun isFinished(): Boolean {
        return controller.atGoal()
    }
}