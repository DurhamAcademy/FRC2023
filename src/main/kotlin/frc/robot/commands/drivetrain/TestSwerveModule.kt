package frc.robot.commands.drivetrain

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.SwerveModule

class TestSwerveModule(
    private var module: SwerveModule,
    private var speed: Double,
) : CommandBase() {
    init {
        addRequirements(module)
    }

    override fun initialize() {
        module.setpoint.speedMetersPerSecond = 2.90
    }

    override fun end(interrupted: Boolean) {
        module.driveMotor.set(0.0)
    }
}