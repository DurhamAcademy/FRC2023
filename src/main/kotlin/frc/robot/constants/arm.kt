package frc.robot.constants

import edu.wpi.first.math.util.Units

object arm {
    object motor {
        const val positionTolerance = 0.05
        const val velocityTolerance = 0.05
        const val id = 34
        const val currentLimit = 40
        const val inverted = true
        const val gearRatio = 1 / 170.67
        const val maxVelocity = 1.0
        const val maxAcceleration = 1.0
        const val kP = 2.6426
        const val kI = 0.0
        const val kD = 1.2626

        const val kS = 0.086989
        const val kG = 0.42677
        const val kV = 3.2483
        const val kA = 0.34796
    }

    object encoder {
        const val id = 18
        const val offset = 6.15//-76.15 + 17 + 25 - 5 - 5 + 77//-87.49
        const val inverted = true
    }

    val length = Units.inchesToMeters(30.31)
    val maxAngle = Units.degreesToRadians(130.0 - 5.0)
    val minAngle = Units.degreesToRadians(-155.0 + 5.0)
    const val armMass = 4.76//lbsToKilograms(10.5)

    // 640.1 in^2 lbs
    const val momentOfInertia = 0.187318642 //kg m^2
}