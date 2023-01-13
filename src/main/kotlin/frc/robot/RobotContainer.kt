package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain

class RobotContainer {
    val xbox = CommandXboxController(0)
    val controlScheme: ControlScheme = DefaultControlScheme(xbox)

    @Suppress("unused")
    val drivetrain = Drivetrain(controlScheme)
}