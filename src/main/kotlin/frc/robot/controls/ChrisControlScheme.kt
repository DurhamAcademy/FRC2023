package frc.robot.controls

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class ChrisControlScheme(
    xboxNum: Int = 0
) : ControlScheme() {
    override val xbox = CommandXboxController(xboxNum)
    override val rotation: Double
        get() = MathUtil.applyDeadband(xbox.rightX.coerceIn(-1.0, 1.0), 0.05)
    override val strafe: Double
        get() = MathUtil.applyDeadband(xbox.leftX.coerceIn(-1.0, 1.0), 0.05)
    override val forward: Double
        get() = MathUtil.applyDeadband(xbox.leftY.coerceIn(-1.0, 1.0), 0.05)
}