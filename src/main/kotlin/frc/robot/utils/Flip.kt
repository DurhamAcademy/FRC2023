package frc.robot.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.kyberlib.math.units.extensions.radians
import frc.robot.Constants
import java.lang.Math.PI

fun flip(p: Pose2d) = Pose2d(
    Constants.Field2dLayout.size.x - p.translation.x,
    p.translation.y,
    p.rotation + Rotation2d.fromRadians(PI)
)