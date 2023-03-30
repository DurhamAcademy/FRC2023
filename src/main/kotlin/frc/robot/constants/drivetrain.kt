package frc.robot.constants

import kotlin.math.PI

object drivetrain {
    const val maxAcceleration = 12.0
    const val maxAngularAcceleration = 3.0 * 2 * PI

    const val maxVelocity = 4.0
    const val maxAngularVelocity = 2.0 * 2 * PI

    const val maxAutonomousVelocity = maxVelocity
    const val maxAutonomousAngularVelocity = maxAngularVelocity

    const val maxAutonomousAcceleration = maxAcceleration * .75
    const val maxAutonomousAngularAcceleration = maxAngularAcceleration * .75
}