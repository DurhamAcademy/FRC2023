package frc.robot.constants

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI

object Constants {

    init {
        SmartDashboard.setDefaultBoolean("Full DS Control", false)
    }

    var fullDSControl: Boolean
        get() = SmartDashboard.getBoolean("Full DS Control", false)
        set(value: Boolean) {
            SmartDashboard.putBoolean("Full DS Control", value)
        }

    val simArmLength = inchesToMeters(10.0)
    const val minAngle = -PI / 2
    const val maxAngle = PI / 2
    const val armMass = 1.0
    const val momentOfInertia = 1.0

    const val powerPercent = 1.0

    const val BRZeroAngle = -10.0 - 5
    const val BLZeroAngle = -29.7 + 2.3
    const val FRZeroAngle = -102.2 - 3.7
    const val FLZeroAngle = -47.9 + 1.3
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

    const val WHEEL_RADIUS = .0497
    const val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS * 2 * PI
    const val DRIVE_GEAR_RATIO = 6.75

    const val MODULE_DISTANCE_X = 0.641
    const val MODULE_DISTANCE_Y = 0.539750

    val cadToCode = Transform3d(
        Translation3d(0.0, 0.029, 0.0),
        Rotation3d(0.0, 0.0, PI / 2)
    )

    const val DRIVE_P = 2.37
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    const val driveKS = 0.20285
    const val driveKV = 2.2335
    const val driveKA = 0.34271

    const val ANGLE_P = 4.1487 //fixme pid
    const val ANGLE_I = 0.0
    const val ANGLE_D = .14002

    const val angleKS = 0.25928
    const val angleKV = 0.28217
    const val angleKA = 0.0050137
}
