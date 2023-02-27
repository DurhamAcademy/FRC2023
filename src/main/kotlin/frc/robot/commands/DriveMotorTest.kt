package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.ControlScheme
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.SwerveModule
import frc.robot.subsystems.Drivetrain

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

    /**
     * prints out different values for controlScheme
     */
    override fun initialize() {
        println(controlScheme)
        println(controlScheme.driveTest.asBoolean)
        println(controlScheme.turnTest.asBoolean)
        println(controlScheme.testPercent)

    }

    /**
     * Sets driveMotor to controlScheme.testPercent
     */

    override fun execute() {
//        modules.forEach {
//            it.driveMotor.set(controlScheme.testPercent)
//        }
    }

    /**
     * Sets driveMotor to 0
     * @param interrupted boolean to process to check if the robot process has been interrupted
     */
    override fun end(interrupted: Boolean) {
//        modules.forEach {
//            it.driveMotor.set(0.0)
//        }
    }
}