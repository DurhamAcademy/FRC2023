package frc.robot

import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.math.util.Units.lbsToKilograms
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import frc.robot.utils.Area
import kotlin.math.PI

object Constants {

    object Elevator {
        val carriageMass = lbsToKilograms(27.5) // rough estimate (25-30 lbs)
        val encoderDistancePerPulse: Double = 1.0 / 4_096.0
        val sproketRadius = inchesToMeters(1.25 / 2.0)

        object elevatorMotor {
            object Feedforward {
                val kG: Double = 1.0

                // a good starting point for kS is 1 / max voltage
                val kS = 1.0 / 12.0
                val kV = 1.0 / 12.0
                val kA = 1.0 // 0.0
            }

            val encoderDistancePerPulse: Double = 1.0 / 4_096.0
            val gearRatio: Double = 170.0

            object PID {
                val kP = 10_000.0
                val kI = 0.0
                val kD: Double = 1000.0

                object TrapezoidProfile {
                    val maxVelocity: Double = 1.0
                    val maxAcceleration: Double = 1.0
                }
            }

            val ElevatorMotorId: Int = 32
        }

        object limitSwitch {
            val ElevatorLimitSwitchId: Int = 0
            val offset: Double = 0.0
        }

        object limits {
            val bottomLimit: Double = inchesToMeters(20.0)
            val topLimit: Double = inchesToMeters(45.0)
        }
    }
    const val maxDriveAcceleration = 3.0
    const val powerPercent = .2

    const val BRZeroAngle = 171.75
    const val BLZeroAngle = 70.25
    const val FRZeroAngle = 76.1
    const val FLZeroAngle = 132.5
    const val FRDriveMotorId = 10//fr
    const val BLDriveMotorId = 11//bl
    const val FLDriveMotorId = 12//fl
    const val BRDriveMotorId = 13

    const val FRTurnMotorId = 14
    const val BLTurnMotorId = 15
    const val FLTurnMotorId = 16
    const val BRTurnMotorId = 17

    const val FRTurnEncoderId = 6
    const val BLTurnEncoderId = 7
    const val FLTurnEncoderId = 8
    const val BRTurnEncoderId = 9

    const val WHEEL_RADIUS = .0508
    const val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS * 2 * PI
    const val DRIVE_GEAR_RATIO = 6.75 * 10

    const val MODULE_DISTANCE_X = 0.641
    const val MODULE_DISTANCE_Y = 0.539750

    val cadToCode = Transform3d(
        Translation3d(0.0, 0.029, 0.0),
        Rotation3d(0.0, 0.0, PI / 2)
    )

    object VisionConstants {
        const val cameraName: String = "OV9281"
        val robotToCam: Pose3d = Pose3d(
            Translation3d(0.0, inchesToMeters(10.188), inchesToMeters(5.433)),
            Rotation3d(
                0.0,
                degreesToRadians(100.0),
                degreesToRadians(90.0)
            )
        ).transformBy(cadToCode)
    }

    @Suppress("unused") //TODO: Remove this suppression
    object FieldConstants {
        const val width = 3.048
        const val length = 5.486
    }

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

    @Suppress("unused") //TODO: Remove this suppression
    object Field2dLayout {
        val bounds = listOf(
            // judge side red -> judge side blue -> far side blue -> far side red
            Translation3d(8.25, -4.0, 0.0),
            Translation3d(-8.25, -4.0, 0.0),
            Translation3d(-8.25, 4.0, 0.0),
            Translation3d(8.25, 4.0, 0.0)
        )
        val center = Translation3d(0.0, 0.0, 0.0)

        object Areas {
            val blueChargingStation = Area(
                Translation3d(-3.5, 0.0, 0.0),
                Translation3d(-5.5, 0.0, 0.0),
                Translation3d(-5.5, 2.5, 0.0),
                Translation3d(-3.5, 2.5, 0.0)
            )
            val redChargingStation = Area(
                Translation3d(3.5, 0.0, 0.0),
                Translation3d(5.5, 0.0, 0.0),
                Translation3d(5.5, 2.5, 0.0),
                Translation3d(3.5, 2.5, 0.0)
            )
            val blueCommunityZone = Area(
                Translation3d(-5.0, -1.5, 0.0),
                Translation3d(-5.0, -2.75, 0.0),
                Translation3d(-1.6, -2.75, 0.0),
                Translation3d(-1.6, -4.0, 0.0),
                Translation3d(-8.25, -4.0, 0.0),
                Translation3d(-8.25, -1.5, 0.0)
            )
            val redCommunityZone = Area(
                Translation3d(5.0, -1.5, 0.0),
                Translation3d(5.0, -2.75, 0.0),
                Translation3d(1.6, -2.75, 0.0),
                Translation3d(1.6, -4.0, 0.0),
                Translation3d(8.25, -4.0, 0.0),
                Translation3d(8.25, -1.5, 0.0)
            )

            val blueScoringZone = Area(
                Translation3d(-5.0, -1.4, 0.0),
                Translation3d(-8.25, -1.4, 0.0),
                Translation3d(-8.25, -1.4, 0.0),
                Translation3d(-8.25, -4.0, 0.0),
                Translation3d(-3.4, -4.0, 0.0),
                Translation3d(-3.4, 0.0, 0.0),
                Translation3d(-5.0, 0.0, 0.0)
            )
            val redScoringZone = Area(
                Translation3d(5.0, -1.4, 0.0),
                Translation3d(8.25, -1.4, 0.0),
                Translation3d(8.25, -1.4, 0.0),
                Translation3d(8.25, -4.0, 0.0),
                Translation3d(3.4, -4.0, 0.0),
                Translation3d(3.4, 0.0, 0.0),
                Translation3d(5.0, 0.0, 0.0)
            )
        }
    }
}