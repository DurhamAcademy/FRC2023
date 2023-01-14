package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.Drivetrain

class RobotContainer {
    val xbox = CommandXboxController(0)

    // subsystems go here:
    val drivetrain = Drivetrain()
}