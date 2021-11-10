package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.Telemetry

class Extensions {

    //I love extension functions, you love extension functions, we all love extension functions!
    companion object {
        fun Telemetry.sendLine(message: String) {
            addLine(message)
            update()
        }
    }
}