package frc.robot.constants

import kotlin.math.PI

object drivetrain {
    const val maxAcceleration = 48.0
    const val maxAngularAcceleration = 5.0 * 2 * PI

    const val maxVelocity = 4.0
    const val maxAngularVelocity = 2.0 * 2 * PI

    const val maxAutonomousVelocity = maxVelocity * .75
    const val maxAutonomousAngularVelocity = maxAngularVelocity * .75

    const val maxAutonomousAcceleration = maxAcceleration * .85
    const val maxAutonomousAngularAcceleration = maxAngularAcceleration * .75
}