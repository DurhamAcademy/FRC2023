package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Intake


open class SetIntakePosition(
    val intake: Intake,
    /** deploy angle in radians */
    val deployAngle: Double,
    /** cube arm angle in radians */
    val cubeArmAngle: Double,
    /** intake speed in percent */
    val intakePercentage: Double,
) : CommandBase() {
    constructor(
        robotContainer: RobotContainer,
        deployAngle: Double,
        cubeArmAngle: Double,
        intakePercentage: Double
    ) : this(
        robotContainer.intake,
        deployAngle,
        cubeArmAngle,
        intakePercentage
    )

    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.setDeployAngle(deployAngle)
        intake.setModeAngle(cubeArmAngle)
        intake.intakePercentage = intakePercentage
    }

    override fun isFinished() =
        intake.deployPID.atGoal() &&
        intake.modePID.atGoal()
}