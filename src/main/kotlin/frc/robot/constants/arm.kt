package frc.robot.constants

import edu.wpi.first.math.util.Units

object arm {
    object motor {
        const val positionTolerance = 0.08
        const val velocityTolerance = 0.05
        const val id = 34
        const val currentLimit = 40
        const val inverted = true
        const val gearRatio = 1 / 152.65
        const val maxVelocity = 2.0
        const val maxAcceleration = 1.2
        const val kP = 3.0
        const val kI = 0.0
        const val kD = 0.5

        const val kS = 0.14018
        const val kG = 0.40234
        const val kV = 3.2121
        const val kA = 0.08093
    }

    object encoder {
        const val id = 18
        const val offset = -57.29 + 6.5 + 2.0//-76.15 + 17 + 25 - 5 - 5 + 77//-87.49
        const val inverted = true
    }

    val length = Units.inchesToMeters(48.0)
    val maxAngle = Units.degreesToRadians(130.0 - 5.0)
    val minAngle = Units.degreesToRadians(-155.0 + 5.0)
    const val armMass = 4.76//lbsToKilograms(10.5)

    // 640.1 in^2 lbs
    const val momentOfInertia = 0.187318642 //kg m^2
}