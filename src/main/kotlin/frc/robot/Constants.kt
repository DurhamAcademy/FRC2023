package frc.robot

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import kotlin.math.PI

object Constants {
    const val BRZeroAngle = -10.0
    const val BLZeroAngle = -29.7
    const val FRZeroAngle = -102.2
    const val FLZeroAngle = -47.9
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
    const val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS * 2 * PI
    const val DRIVE_GEAR_RATIO = 6.75

    const val MODULE_DISTANCE_X = 0.641
    const val MODULE_DISTANCE_Y = 0.539750

    object VisionConstants {
        val robotToCam: Transform3d = Transform3d(Translation3d(), Rotation3d())
        const val cameraName: String = "OV9281"
    }

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
}