package frc.robot.utils

enum class GamePiece(val deployAngle: Double, val cubeArmAngle: Double) {
    cone(0.0, 0.0),
    cube(0.0, 0.0),
    unknown(0.0, 0.0),
    none(0.0, 0.0)
}