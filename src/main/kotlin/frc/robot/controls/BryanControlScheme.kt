package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

class BryanControlScheme(
    xboxNum: Int = 0
) : ControlScheme() {
    override val xbox = CommandXboxController(xboxNum)

    override val lowIntake = xbox.leftTrigger()
    override val intakeHPS = xbox.rightTrigger()

    override val spinIntakeIn = xbox.leftBumper()
    override val spinIntakeOut = xbox.rightBumper()

    override val idleConfiguration = xbox.x()
    override val placeLvl2 = xbox.b()
    override val placeLvl3 = xbox.y()

    override val ledColor = xbox.povLeft().or(xbox.povUpLeft()).or(xbox.povDownLeft())

    override val throwObject: Trigger = xbox.rightBumper()
    override val intakeGroundIntake = xbox.povUp().or(xbox.povUpLeft()).or(xbox.povUpRight())
    override val intakeEject = xbox.a()

    override val shootToLTwo: Trigger = xbox.back()
    override val shootToLThree: Trigger = xbox.start()
}
