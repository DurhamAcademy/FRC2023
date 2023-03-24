package frc.robot.constants

import edu.wpi.first.math.util.Units

object elevator {
    val carriageMass = Units.lbsToKilograms(27.5) // rough estimate (25-30 lbs)
    val encoderDistancePerPulse: Double = 1.0 / 2_048.0
    val sproketRadius = Units.inchesToMeters(1.273 / 2.0)

    object elevatorMotor {
        object Feedforward {
            val kG: Double = 0.092925 //0.047
            val kS = 0.16
            val kV = 11.477
            val kA = 0.18208
        }

        object tolerance {
            const val positionTolerance: Double = 0.0012075
            const val velocityTolerance: Double = 0.002
        }
        //todo add encoder offset to smartdashboard

        val inverted = true
        val gearRatio: Double = 1 / 10.51

        object PID {
            val kP = 55.0//892.53
            val kI = 0.0
            val kD: Double = 3.0//9.4036

            object TrapezoidProfile {
                val maxVelocity: Double = 1.0
                val maxAcceleration: Double = 1.0
            }
        }

        val ElevatorMotorId: Int = 32
    }

    object limitSwitch {
        val ElevatorLimitSwitchId: Int = 0
        val offset: Double = limits.bottomLimit
    }

    object limits {
        val bottomLimit: Double = Units.inchesToMeters(17.83)
        val topLimit: Double = Units.inchesToMeters(48.5)
    }
}