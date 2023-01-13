package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import kotlin.math.absoluteValue
import kotlin.math.pow

class DefaultControlScheme(
    val xboxController: CommandXboxController,
) : ControlScheme {
    override val rotation: Double // added deadband
        get() = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it.pow(3) }
    override val strafe: Double
        get() = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it.pow(3) }
    override val forward: Double
        get() = xboxController.rightY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it.pow(3) }
}