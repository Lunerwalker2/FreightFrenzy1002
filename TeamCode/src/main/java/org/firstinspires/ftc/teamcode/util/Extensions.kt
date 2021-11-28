package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry

class Extensions {




    //I love extension functions, you love extension functions, we all love extension functions!
    companion object {

        @JvmField
        var HEADING_SAVER: Double = 0.0

        fun Telemetry.sendLine(message: String) {
            addLine(message)
            update()
        }


        fun toFieldRelative(original: Pose2d, angleRad: Double): Pose2d {
            val vec = original.vec().rotated(-angleRad)

            return Pose2d(vec, original.heading)

        }
    }


}