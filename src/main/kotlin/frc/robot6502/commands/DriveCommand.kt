package frc.robot6502.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot6502.controls.ControlScheme
import frc.robot6502.subsystems.Drivetrain

class DriveCommand(
    var drivetrain: Drivetrain,
    var controlScheme: ControlScheme,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        drivetrain.drive(
            ChassisSpeeds(
                controlScheme.forward,
                controlScheme.strafe,
                controlScheme.rotation
            ),
            false
        )
    }
}