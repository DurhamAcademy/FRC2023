package frc.robot.commands.drivetrain

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class DriveCommand(
    val drivetrain: Drivetrain,
    inline val x: () -> Double = { 0.0 },
    inline val y: () -> Double = { 0.0 },
    inline val rotation: () -> Double = { 0.0 },
    val isFieldOriented: Boolean = true,
    inline val centerOffset: () -> Translation2d = { Translation2d(0.0, 0.0) }
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        drivetrain.drive(
            ChassisSpeeds(x(), y(), rotation()),
            true,
            Translation2d(0.0, 0.0)
        )
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(
            ChassisSpeeds(0.0, 0.0, 0.0),
            true,
            Translation2d(0.0, 0.0)
        )
    }
}