package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.util.Range
import kotlin.math.cos

class ArmPIDFController {


    val armController = PIDFController(PIDCoefficients(6.0, 0.0, 0.0), kF = { // 0 to bottom 43 ticks, 1.48 degs per tick
        position: Double, _ ->
            1.6 * cos(position / 420 * 360) //gravity feedforward term is Fg * cos(angle of arm)
    })

    /*
    See tylers book on controls engineering in frc.

    Fg = mg(L/2)
     */
}