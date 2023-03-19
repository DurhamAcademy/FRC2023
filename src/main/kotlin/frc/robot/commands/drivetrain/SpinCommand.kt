package frc.robot.commands.drivetrain

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI

class SpinCommand(
    var drivetrain: Drivetrain
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        drivetrain.drive(
            ChassisSpeeds(0.0, 0.0, PI),
            true,
        )
    }
}