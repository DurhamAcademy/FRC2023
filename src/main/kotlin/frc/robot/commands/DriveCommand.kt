package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain

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
                controlScheme.forward * 2.0,
                controlScheme.strafe * 2.0,
                controlScheme.rotation * 2 * Math.PI
            ),
            false
        )
    }
}