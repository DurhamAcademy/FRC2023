package frc.robot.controls

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

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

    override val alignClosestConeL1 = Trigger { false }
    override val confirmGridSelection: Trigger = xbox.a()
    override val alignClosestConeL2: Trigger = xbox.x()
    override val alignClosestConeL3: Trigger = xbox.y()
    override val alignClosestHPS: Trigger = xbox.b()

    override val decreaseEncoderAngle: Trigger = xbox.povLeft()
    override val increaseEncoderAngle: Trigger = xbox.povRight()
}