package frc.robot.commands.drivetrain

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.SwerveModule

class DriveMotorTest(
    private val drivetrain: Drivetrain,
    val controlScheme: ControlScheme
) : CommandBase() {
    val modules: List<SwerveModule>
        get() {
            return drivetrain.modules
        }

    init {
        addRequirements(drivetrain)
    }

    override fun initialize() {
        println(controlScheme)

    }

    override fun execute() {
//        modules.forEach {
//            it.driveMotor.set(controlScheme.testPercent)
//        }
    }

    override fun end(interrupted: Boolean) {
//        modules.forEach {
//            it.driveMotor.set(0.0)
//        }
    }
}