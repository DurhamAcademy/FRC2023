package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Intake

class SetIntakePosition(
    val intake: Intake,
    val intakePosition: Double,
) : CommandBase() {
    constructor(
        robotContainer: RobotContainer,
        intakePosition: Double
    ) : this(
        robotContainer.intake,
        intakePosition
    )

    companion object {
        fun deploy(intake: Intake) = SetIntakePosition(
            intake, 0.0 //TODO Change this to actual pos
        )

        fun retract(intake: Intake) = SetIntakePosition(
            intake, 0.0 //TODO Change this to actual pos
        )
    }
    init {
        addRequirements(intake)
    }

    override fun execute() {
        intake.setIntakePosition(intakePosition)
    }
}