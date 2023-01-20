package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class NormalControlScheme(val xboxController: CommandXboxController) : ControlScheme {
    override val forward: Double
        get() = xboxController.rightY
    override val strafe: Double
        get() = xboxController.rightX
    override val rotation: Double
        get() = xboxController.leftX
}