package frc.robot.commands.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer

class rotateTo180(
    val robotContainer: RobotContainer,
) : CommandBase() {
    override fun execute() {
        robotContainer.rotateTo180 = true
    }
    override fun end(interrupted: Boolean) {
        robotContainer.rotateTo180 = false
    }
}