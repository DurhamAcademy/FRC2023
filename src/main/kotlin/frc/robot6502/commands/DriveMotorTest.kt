package frc.robot6502.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot6502.subsystems.SwerveModule
import frc.robot6502.subsystems.Drivetrain

class DriveMotorTest(
    private val drivetrain: Drivetrain,
    private val module: SwerveModule?,
    private val speed: Double,
) : CommandBase() {
    val modules: List<SwerveModule>
        get() {
            return if (module == null) {
                drivetrain.modules
            } else {
                listOf(module)
            }
        }

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
        modules.forEach {
            it.driveMotor.set(speed)
        }
    }

    override fun end(interrupted: Boolean) {
        modules.forEach {
            it.driveMotor.set(0.0)
        }
    }
}