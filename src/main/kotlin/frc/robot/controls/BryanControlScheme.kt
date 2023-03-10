package frc.robot.controls

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.absoluteValue

class BryanControlScheme(
    xboxNum: Int = 0
) : ControlScheme() {
    override val xbox = CommandXboxController(xboxNum)
    override val rotation: Double
        get() = MathUtil.applyDeadband(xbox.rightX.coerceIn(-1.0, 1.0), 0.05)
    override val strafe: Double
        get() = MathUtil.applyDeadband(xbox.leftX.coerceIn(-1.0, 1.0), 0.05)
    override val forward: Double
        get() = MathUtil.applyDeadband(xbox.leftY.coerceIn(-1.0, 1.0), 0.05)

    override val idleConfiguration = xbox.x()
    override val placeLvl1 = xbox.a()
    override val placeLvl2 = xbox.b()
    override val placeLvl3 = xbox.y()

    override val intake = xbox.leftBumper()
    override val outtake = xbox.rightBumper()

    override val lowIntake = xbox.leftTrigger()
    override val highIntake = xbox.rightTrigger()

//    override val toggleManipulator = xbox.povUp()
    override val toggleManipulator: Trigger
        get() = xbox.povUp()


    override val moveToClosestHPS: Trigger = xbox.start()
    override val moveToClosestScoreStation: Trigger = xbox.back()
}
