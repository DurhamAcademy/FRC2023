package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece

class IdleIntake (
    private val intake: Intake,
    val gamePiece: () -> GamePiece,
) : CommandBase() {

    init{
        addRequirements(intake)
    }

    override fun execute() {
        when (gamePiece()) {
            //SWAPPED
            GamePiece.cube -> {
                intake.intakePercentage = -0.15
                intake.modeVoltage = -3.0
                intake.setDeployAngle(0.3)
                intake.setModeAngle(2.25)

            }

            GamePiece.cone -> {
                intake.intakePercentage = -0.15
                intake.modeVoltage = -3.0
                intake.setDeployAngle(0.3)
                intake.setModeAngle(0.3)
            }

            else -> {
                intake.intakePercentage = -0.15
                intake.modeVoltage = -3.0
                intake.setDeployAngle(0.3)
                intake.setModeAngle(0.3)
            }
        }
    }

    override fun isFinished() = false
}

