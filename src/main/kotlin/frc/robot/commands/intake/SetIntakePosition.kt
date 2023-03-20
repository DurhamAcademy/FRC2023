package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.utils.grid.PlacementLevel
import kotlin.math.absoluteValue
import kotlin.math.sin

class SetIntakePosition(
    val intake: Intake,
    val intakePosition: Double,
) : CommandBase() {
    constructor(robotContainer: RobotContainer,
                intakePosition: Double) : this(
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