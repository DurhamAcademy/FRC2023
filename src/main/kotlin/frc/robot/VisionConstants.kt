package frc.robot

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

object VisionConstants {
    const val cameraName: String = "OV9281"
    val robotToCam: Transform3d = Transform3d(
        Translation3d(-0.258, 0.0, 0.137),
        Rotation3d(
            0.0,
            Units.degreesToRadians(-12.0),
            Units.degreesToRadians(180.0)
        )
    )
}