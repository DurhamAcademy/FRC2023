package frc.robot.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

@Suppress("unused") //TODO: Remove this suppression
class BoundingBox(
    val x: Double,
    val y: Double,
    val z: Double = 0.0,
    val width: Double,
    val height: Double,
    val depth: Double = 4.0,
) {
    val area: Double
        get() = width * height
    val centerX: Double
        get() = x + width / 2
    val centerY: Double
        get() = y + height / 2

    fun expanded(by: Double) = BoundingBox(
        x - by,
        y - by,
        z,
        width + by * 2,
        height + by * 2,
        depth
    )

    fun contains(point: Translation2d) =
        (
                point.x in x..x + width
                        && point.y in y..y + height
                )

    fun contains(point: Translation3d) =
        (
                point.x in x..x + width
                        && point.y in y..y + height
                        && point.z in z..z + depth
                )
}