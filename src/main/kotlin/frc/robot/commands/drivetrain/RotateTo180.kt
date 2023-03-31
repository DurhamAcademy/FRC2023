package frc.robot.commands.drivetrain

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer

class RotateTo180(
    val robotContainer: RobotContainer,
) : CommandBase() {
    override fun execute() {
        robotContainer.rotateTo180 = true
    }
    override fun end(interrupted: Boolean) {
        robotContainer.rotateTo180 = false
    }
}