package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import kotlin.math.absoluteValue

class BryanControlScheme : ControlScheme() {
    override val xbox = CommandXboxController(0)
    override val rotation: Double
        get() = xbox.rightX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
    override val strafe: Double
        get() = xbox.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
    override val forward: Double
        get() = xbox.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }

    override val idleConfiguration = xbox.x()
    override val placeLvl1 = xbox.a()
    override val placeLvl2 = xbox.b()
    override val placeLvl3 = xbox.y()

    override val intake = xbox.leftBumper()
    override val outtake = xbox.rightBumper()

    override val lowIntake = xbox.leftTrigger()
    override val highIntake = xbox.rightTrigger()

    override val openManipulator = xbox.povUp()
    override val closeManipulator = xbox.povDown()
}