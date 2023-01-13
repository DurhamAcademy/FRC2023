package frc.robot.controls

interface ControlScheme {
    val rotation: Double
    val strafe: Double
    val forward: Double
}