package frc.robot

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import kotlin.math.PI

object Constants {
    const val gyroReversed = false
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

    object VisionConstants {
        val robotToCam: Transform3d = Transform3d(Translation3d(), Rotation3d())
        const val cameraName: String = "photonvision"
    }

    object FieldConstants {
        const val width = 3.048
        const val length = 5.486
    }
}