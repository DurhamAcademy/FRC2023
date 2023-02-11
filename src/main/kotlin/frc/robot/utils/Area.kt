package frc.robot.utils

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

class Area(
    val depth: Double = 4.0,
    vararg points: Translation2d,
) {
    constructor(vararg points: Translation3d) : this(
        depth = (if (points.isEmpty()) 0.0 else (
                points.maxByOrNull { it.z }?.minus(points.minByOrNull { it.z })
                )?.z ?: 0.0).run { if (this == 0.0) 4.0 else this },
        *points.map { Translation2d(it.x, it.y) }.toTypedArray()
    )

    val points: List<Translation2d> = points.toList()

    @Suppress("unused") //TODO: Remove this suppression
    val boundingBox: BoundingBox
        get() {
            val x = points.minByOrNull { it.x }?.x ?: 0.0
            val y = points.minByOrNull { it.y }?.y ?: 0.0
            val width = points.maxByOrNull { it.x }?.x ?: (0.0 - x)
            val height = points.maxByOrNull { it.y }?.y ?: (0.0 - y)
            return BoundingBox(x, y, depth, width, height)
        }

    fun contains(point: Translation2d) =
        points.mapIndexed { index, translation2d ->
            val next = points[(index + 1) % points.size]
            val a = next.y - translation2d.y
            val b = translation2d.x - next.x
            val c = next.x * translation2d.y - translation2d.x * next.y
            a * point.x + b * point.y + c >= 0
        }.all { it }

    fun contains(point: Translation3d): Boolean {
        if (point.z !in 0.0..depth) return false
        return contains(Translation2d(point.x, point.y))
    }
}