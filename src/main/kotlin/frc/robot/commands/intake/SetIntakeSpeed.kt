package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class SetIntakeSpeed(
    val intake: Intake,
    private val speed: Double
) : CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun execute() {
        intake.driveMotorPercentage = speed
    }
}