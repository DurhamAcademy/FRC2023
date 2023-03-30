package frc.robot.constants

import edu.wpi.first.math.util.Units

object intake {
    const val driveMotorId = 48
    const val driveMotorLimit = 0

    const val systemMotorId = 49 //moves the intake up and down
    const val systemMotorLimit = 0

    const val modeMotorId = 50
    const val modeMotorLimit = 20

    const val maxAngle = 0.0
    const val minAngle = 0.0

//turns whole intake
object deployMotor {
    const val pidOnRio = true
    const val positionTolerance = 0.0
    const val velocityTolerance = 0.0
    const val id = 49
    const val inverted = true
    const val maxVelocity = 1.0
    const val maxAcceleration = 1.0
    const val kP = 4.0
    const val kI = 0.0
    const val kD = 0.0

    const val kS = 0.0
    const val kG = 0.0
    const val kV = 0.0
    const val kA = 0.0
}

    //switches in between cone and cube
    object modeMotor {
        val pidOnRio = true
        const val positionTolerance = 0.0
        const val velocityTolerance = 0.0
        const val id = 0
        const val inverted = true
        const val gearRatio = 1 / 30
        const val maxVelocity = 1.0
        const val maxAcceleration = 1.0
        const val kP = 3.05
        const val kI = 0.0
        const val kD = 0.0

        const val kS = 0.0
        const val kG = 0.0
        const val kV = 0.0
        const val kA = 0.0
    }

    object limitSwitch{
        const val intakeLimitSwitch = 1
    }

    object limits{
        val backlimit: Double = Units.radiansToDegrees(0.0)
        val topLimit: Double = Units.radiansToDegrees(-1.0)
    }
}