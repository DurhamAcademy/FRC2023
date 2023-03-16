package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class BryanControlScheme(
    xboxNum: Int = 0
) : ControlScheme() {
    override val xbox = CommandXboxController(xboxNum)

    override val lowIntake = xbox.leftTrigger()
    override val intakeHPS = xbox.rightTrigger()

    override val spinIntakeIn = xbox.leftBumper()
    override val spinIntakeOut = xbox.rightBumper()

    override val idleConfiguration = xbox.x()
    override val placeLvl1 = xbox.a()
    override val placeLvl2 = xbox.b()
    override val placeLvl3 = xbox.y()

    override val ledColor = xbox.povLeft()

    override val stopIntake = xbox.povUp()
}
