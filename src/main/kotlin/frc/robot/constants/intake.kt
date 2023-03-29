package frc.robot.constants

import edu.wpi.first.math.util.Units

object intake {
    const val driveMotorId = 0 //turns wheels
    const val driveMotorLimit = 0

    const val systemMotorId = 0 //moves the intake up and down
    const val systemMotorLimit = 0

    const val modeMotorId = 0
    const val modeMotorLimit = 0

    const val maxAngle = 0.0
    const val minAngle = 0.0

//turns whole intake
object baseMotor {
    const val positionTolerance = 0.0
    const val velocityTolerance = 0.0
    const val id = 0
    const val inverted = true
    const val gearRatio = 1 / 2
    const val maxVelocity = 0.0
    const val maxAcceleration = 0.0
    const val kP = 0.0
    const val kI = 0.0
    const val kD = 0.0

        const val kS = 0.0
        const val kG = 0.0
        const val kV = 0.0
        const val kA = 0.0
    }

    //switches in between cone and cube
    object modeMotor {
        const val positionTolerance = 0.0
        const val velocityTolerance = 0.0
        const val id = 0
        const val inverted = true
        const val gearRatio = 1 / 120
        const val maxVelocity = 0.0
        const val maxAcceleration = 0.0
        const val kP = 0.0
        const val kI = 0.0
        const val kD = 0.0

        const val kS = 0.0
        const val kG = 0.0
        const val kV = 0.0
        const val kA = 0.0
    }

    object baseEncoder {
        const val id = 0
        const val offset = 0.0
        const val inverted = false
    }

    object modeEncoder {
        const val id = 0
        const val offset = 0.0
        const val inverted = false
    }

    object limitSwitch{
        const val intakeLimitSwitch = 0
    }

    object limits{
        val backlimit: Double = Units.radiansToDegrees(0.0)
        val topLimit: Double = Units.radiansToDegrees(-1.0)
    }
}