package frc.robot

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object Constants {
    object Arm {
        val carriageMass = 10.0
        val encoderDistancePerPulse: Double = 1.0 / 4_096.0
        val gearRatio: Double = 1.0
        val sproketRadius = Units.inchesToMeters(1.0)

        object armMotor {
            val encoderDistancePerPulse: Double = 1.0 / 4_096.0
            val gearRatio: Double = 1.0

            object PID {
                val kP = 48.0
                val kI = 0.0
                val kD: Double = 0.0

                object TrapezoidProfile {
                    val maxVelocity: Double = 1.0
                    val maxAcceleration: Double = 1.0
                }
            }

            object FeedForeward {
                // a good starting point for kS is 1 / max voltage
                val kS = 1.0 / 12.0
                val kV = 0.0
                val kA = 0.0
            }

            val ArmMotorId: Int = 39
        }

        object limits {
            val bottomLimit: Double = 0.0
            val topLimit: Double = 8.0
        }

    }

    object Elevator {
        val carriageMass = 10.0
        val encoderDistancePerPulse: Double = 1.0 / 4_096.0
        val gearRatio: Double = 1.0
        val sproketRadius = Units.inchesToMeters(1.0)

        object elevatorMotor {
            val encoderDistancePerPulse: Double = 1.0 / 4_096.0
            val gearRatio: Double = 1.0

            object PID {
                val kP = 48.0
                val kI = 0.0
                val kD: Double = 0.0

                object TrapezoidProfile {
                    val maxVelocity: Double = 1.0
                    val maxAcceleration: Double = 1.0
                }
            }

            object FeedForeward {
                // a good starting point for kS is 1 / max voltage
                val kS = 1.0 / 12.0
                val kV = 0.0
                val kA = 0.0
            }

            val ElevatorMotorId: Int = 39
        }

        object positions {
            val bottomElevatorTranslation: Translation3d = TODO()
            val topElevatorTranslation: Translation3d = TODO()
        }

        object limits {
            val bottomLimit: Double = 0.0
            val topLimit: Double = 8.0
        }

    }

    const val BRZeroAngle = 171.75
    const val BLZeroAngle = 70.25
    const val FRZeroAngle = 76.1
    const val FLZeroAngle = 132.5
    const val FRDriveMotorId = 10//fr
    const val BLDriveMotorId = 11//bl
    const val FLDriveMotorId = 12//fl
    const val BRDriveMotorId = 13

    const val FRTurnMotorId = 14//fr
    const val BLTurnMotorId = 15//bl
    const val FLTurnMotorId = 16//fl
    const val BRTurnMotorId = 17

    const val FRTurnEncoderId = 6//fr
    const val BLTurnEncoderId = 7//bl
    const val FLTurnEncoderId = 8//fl
    const val BRTurnEncoderId = 9

    const val WHEEL_RADIUS = .0508
    val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS * 2 * PI
    const val DRIVE_GEAR_RATIO = 6.75

    const val MODULE_DISTANCE_X = 0.641
    const val MODULE_DISTANCE_Y = 0.539750

    const val DRIVE_P = 2.37
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    const val driveKS = 0.21862
    const val driveKV = 2.2997
    const val driveKA = 0.26242

    const val ANGLE_P = 0.5
    const val ANGLE_I = 0.0
    const val ANGLE_D = 0.0

    const val angleKS = 0.24233
    const val angleKV = 0.28267
    const val angleKA = 0.0144
}