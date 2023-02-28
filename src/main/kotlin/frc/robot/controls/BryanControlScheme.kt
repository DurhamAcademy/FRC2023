package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

class BryanControlScheme : ControlScheme() {
    override val xbox = CommandXboxController(0)
    override val rotation = xbox.leftX
    override val strafe = xbox.leftY
    override val forward = xbox.rightY

    override val idleConfiguration = xbox.x()
    override val placeLvl1 = xbox.a()
    override val placeLvl2 = xbox.b()
    override val placeLvl3 = xbox.y()

    override val intake = xbox.leftBumper()
    override val outtake = xbox.rightBumper()

    override val lowIntake = xbox.leftTrigger()
    override val highIntake = xbox.rightTrigger()

    override val openManipulator = xbox.back()
    override val closeManipulator = xbox.start()
}