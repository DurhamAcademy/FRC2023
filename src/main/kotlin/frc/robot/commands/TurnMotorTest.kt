package frc.robot.commands

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.SwerveModule
import frc.robot.subsystems.Drivetrain

class TurnMotorTest(
    private val drivetrain: Drivetrain,
    private var module: SwerveModule?,
    private var speed: Double,
    private var controlScheme: DefaultControlScheme
) : CommandBase() {
    val modules: List<SwerveModule>
        get() {
            return if (module == null) {
                drivetrain.modules
            } else {
                listOf(module!!)
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

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder?.addDoubleProperty("speed", { speed }, { speed = it })
        builder?.addBooleanProperty("front left", { module == drivetrain.frontLeft }, { if (it) module = drivetrain.frontLeft })
        builder?.addBooleanProperty("front right", { module == drivetrain.frontRight }, { if (it) module = drivetrain.frontRight })
        builder?.addBooleanProperty("back left", { module == drivetrain.backLeft }, { if (it) module = drivetrain.backLeft })
        builder?.addBooleanProperty("back right", { module == drivetrain.backRight }, { if (it) module = drivetrain.backRight })
    }
}